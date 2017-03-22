package drone;

import java.awt.image.BufferedImage;
import java.util.ArrayList;

import org.opencv.core.Mat;

import billedanalyse.IBilledAnalyse;
import billedanalyse.QRCodeScanner;
import billedanalyse.RetningsVektor;
import billedanalyse.Squares;
import billedanalyse.Squares.FARVE;
import diverse.Log;
import diverse.PunktNavigering;
import diverse.QrFirkant;
import diverse.circleCalc.Vector2;
import diverse.koordinat.Genstand;
import diverse.koordinat.Genstand.GENSTAND_FARVE;
import diverse.koordinat.Koordinat;
import diverse.koordinat.OpgaveRum;
import drone.DroneHelper.DIRECTION;

public class OpgaveAlgoritme2 implements Runnable {

	/*
	 * Markør hvor der kan udskrives debug-beskeder i konsollen.
	 */
	protected final boolean OPGAVE_DEBUG = true;

	private IDroneControl dc;
	private IBilledAnalyse ba;
	private OpgaveRum opgrum;
	private DroneHelper dh;
	private PunktNavigering punktNav;
	private QRCodeScanner qrcs;
	private PunktNavigering pn;
	private RetningsVektor rv;
	private boolean doStop = false;
	private boolean flying = false;
	private ArrayList<Koordinat> searchPoints, objectCoords;
	private ArrayList<Squares> squarePoints;
	private Koordinat landingsPlads, baneStart, baneSlut;
	private long stopTid, startTid;
	private int yaw;
	private int xOffset = 0;
	private int yOffset = 0;


	public OpgaveAlgoritme2(IDroneControl dc, IBilledAnalyse ba, OpgaveRum opgrum){
		Log.writeLog("Opgavealgoritme oprettet");
		this.dc = dc;
		this.ba = ba;
		this.punktNav = new PunktNavigering();
		//		this.createPoints();
		this.opgrum = opgrum;
		this.dh = new DroneHelper(dc, opgrum.getWidth(), opgrum.getLength());
	}

	/**
	 * WARNING - USE AT OWN RISK
	 * This method will attempt to start SKYNET
	 * WARNING - USE AT OWN RISK
	 * @throws InterruptedException 
	 */
	public void startOpgaveAlgoritme() throws InterruptedException {
		Log.writeLog("*** OpgaveAlgoritmen startes. ***");
		dc.setTimeMode(true);
		dc.setFps(30); // TODO - Giver 30 frames problemer med WIFI-forbindelsen?
		while (!doStop) {
			if(Thread.interrupted()){
				destroy();
				return;
			}
			boolean img = false;
			while(!flying){
				if(Thread.interrupted()){
					destroy();
					return;
				}
				// Hvis dronen ikke er klar og videostream ikke er tilgængeligt, venter vi 500 ms mere
				if(!dc.isReady() || (img = ba.getImages() == null)){
					if(ba.getImages()==null){
						img = false;						
					}
					if(OPGAVE_DEBUG){
						System.out.println("Drone klar: " + dc.isReady()+ ", Billeder modtages: " + img);					
					}
					try {
						Thread.sleep(500);
					} catch (InterruptedException e) {
						destroy();
						return;
					}
					continue; // start forfra i while-løkke
				} else {// Dronen er klar til at letter
					System.err.println("*** WARNING - SKYNET IS ONLINE");
					Log.writeLog("*Dronen letter autonomt.");
					flying = true;
					dc.takeoff();

					if(OPGAVE_DEBUG){
						System.err.println("*** Dronen letter og starter opgaveløsning ***");
					}
					try {
						Thread.sleep(5000);
					} catch (InterruptedException e) {
						destroy();
						return;
					}
				}
			}
			

			this.opgave2(); // Afsøg opgaverummet
//						this.testFlight();
			destroy(); // Land dronen
		}
	}

	//	/** Modtager et array af squares der gennemløbes og sendes videre til behandling i OpgaveRum,
	//	 *  som returnerer et koordinat for hver square der gennemløbes 
	//	 *  i square-arrayet. Herefter gennemløbes koordinat-arrayet og tjek for objekternes position udføres 
	//	 *  og sendes til OpgaveRum så de kan ses i GUI'en
	//	 * @param squares
	//	 */
	//	private void getSquaresPositioner(ArrayList<Squares> squares) {
	//		stopTid = System.currentTimeMillis();
	//		Koordinat dronePos = baneStart;
	//
	//		//		double dist = baneStart.dist(baneSlut); // Hvor langt har dronen bevæget sig?
	//
	//		// Opdater dronepositionen med tiden og retningen siden dronen sidst opdaterede sin position
	//		for(Squares item: squares) {
	//			//			long squaresdif = startTid - item.getTid();
	//			//			int afstand = (int) ((stopTid - startTid)/(squaresdif*dist)); // (DeltaTid / tid) * distance
	//
	//			//			int[] data = dc.getFlightData();
	//			Vector2 vector = rv.getVector(ba.getVektorArray(), yaw);
	//			//			int afstand2 = dronePos.dist(rv.vectorFromStart(dronePos, vector));
	//
	//			if (baneStart.getY() < 500) {
	//				//				dronePos.setY(baneStart.getY() + afstand2);
	//				dronePos = rv.vectorFromStart(baneStart, dronePos.getVector().add(vector));
	//			} else if (baneStart.getY() > 500) {
	//				//				dronePos.setY(baneStart.getY() - afstand2);
	//				dronePos = rv.vectorFromStart(baneStart, dronePos.getVector().add(vector));
	//			}
	//
	//			rv.vectorFromStart(baneStart, dronePos.getVector().add(vector));
	//
	//			Koordinat objectcoord = opgrum.rotateCoordinate(item, dronePos, dc.getFlightData()[3]);
	//			objectCoords.add(objectcoord);
	//		}	
	//
	//		// Tilføj de fundne objekter i rummets koordinater til opgaverummet
	//		for (int i = 0; i < objectCoords.size(); i++) {
	//			Koordinat item = objectCoords.get(i);
	//			for(Koordinat k : objectCoords){
	//				//Tjek for om objektet ikke ligger for tæt på objektet før
	//				if(!k.equals(item) && item.dist(k) > 10){
	//					Genstand genstand;
	//					if(squares.get(i).getFarve() == FARVE.RØD){
	//						genstand = new Genstand(GENSTAND_FARVE.RØD);
	//					} else {
	//						genstand = new Genstand(GENSTAND_FARVE.GRØN);
	//					}
	//					opgrum.addGenstandTilKoordinat(item, genstand);						
	//				}
	//			}
	//		} 
	//	}

	//	private void createPoints(){
	//		int yMax = 900; // Max Y værdi i koordinatsættet som dronen skal besøge
	//		int yMin = 0; // Min Y værdi i koordinatsættet som dronen skal besøge
	//		int xMin = 478; // Min X værdi i koordinatsættet som dronen skal besøge
	//		int step = 75; // Bredden af en søgebane
	//		searchPoints = new ArrayList<Koordinat>();
	//		for(int i=0; i<8; i++){
	//			if(i%2==0){
	//				searchPoints.add(new Koordinat(xMin + i*step, yMin));
	//				searchPoints.add(new Koordinat(xMin + i*step, yMax));
	//			}else {
	//				searchPoints.add(new Koordinat(xMin + i*step, yMax));
	//				searchPoints.add(new Koordinat(xMin + i*step, yMin));
	//			}
	//		}
	//	}
	//
	//	private void objectSearch() throws InterruptedException{	
	//		final int ACCEPT_DIST = 100; // Acceptabel fejlmargin i position (cm)
	//
	//		try{
	//			Thread.sleep(3000);
	//
	//			// Find dronepos
	//			Koordinat dronePos = findDronePos();
	//			baneStart = dronePos;
	//
	//			// Roter drone (mod vinduet)
	//			Log.writeLog("Drejer dronen til YAW = 0");
	//			dc.turnDroneTo(0);
	//
	//			// Strafe højre/venstre
	//			for(int i=0; i<1; i++){ // TODO antal baner
	//				//Skift kamera (nedaf)
	//				dc.toggleCamera();
	//				Thread.sleep(3000);
	//
	//				startTid = System.currentTimeMillis();
	//				ba.setObjTrack(true); // Track objekter
	//				Log.writeLog("ObjektTracking startes: " + startTid);
	//				yaw = dc.getFlightData()[2];
	//				// Strafe 90%
	//				dh.strafePunkt(searchPoints.get(2*i), searchPoints.get(2*i+1));				
	//				ba.setObjTrack(false);
	//				stopTid = System.currentTimeMillis();
	//				Log.writeLog("ObjektTracking stoppes: " + stopTid);
	//				Log.writeLog("Objekter fundet: " + ba.getColorSquares().size());
	//
	//				//Skift kamera (fremad)
	//				dc.toggleCamera();
	//				Thread.sleep(3000);
	//				// Tjek pos
	//				dronePos = findDronePos();
	//				baneSlut = dronePos;
	//				Log.writeLog("Tegner objekter på GUI.");
	//				getSquaresPositioner(ba.getColorSquares());
	//				if(dronePos.dist(searchPoints.get(2*i+1)) > ACCEPT_DIST){
	//					// Finjuster til næste startbane
	//					if(i!=7){					
	//						dh.adjust(dronePos, searchPoints.get(2*i+2));
	//					}
	//				}
	//			}
	//		} catch (NullPointerException e){
	//			Log.writeLog("Drone position ej fundet. Lander.");
	//			destroy();
	//			return;
	//		}
	//	}

	//	private Koordinat findDronePos() throws InterruptedException{
	//		Thread.sleep(2000);
	//		boolean posUpdated = false;
	//		long start = System.currentTimeMillis();
	//		Koordinat drone;
	//		//		setDroneHeight(ba.getFirkant());
	//		if((drone = ba.getDroneKoordinat()) != null){
	//			Log.writeLog("Drone position fundet: " + drone.toString());
	//			return drone;
	//		} else { 
	//			int startYaw = dc.getFlightData()[2];
	//			int turns = 0;
	//			while(!posUpdated && turns < 4 ){
	//				Log.writeLog("Drejer dronen: " + (System.currentTimeMillis() - start)/1000);
	//				dc.turnLeft();
	//				turns++;
	//				dc.hover();
	//				Thread.sleep(5000); // Vent på dronen udfører kommandoen og vi får et rent billede
	//				if((drone = ba.getDroneKoordinat()) != null){
	//					posUpdated = true;
	//					Log.writeLog("Drone position fundet: " + drone.toString());
	//					return drone;
	//					//					dc.turnDroneTo(startYaw); // Drej dronen tilbage til startpositionen
	//				}
	//			}
	//		}
	//		return drone;
	//	}

	private void modeSwitchQr() throws InterruptedException{
		ba.setObjTrack(false);
		dc.toggleCamera();
		Log.writeLog("KAMERA: Skifter til fremadrettet kamera");			
		Thread.sleep(2000);
		ba.setDroneLocator(true);
		//		ba.setOpticalFlow(false);
	}

	private void modeSwitchObject() throws InterruptedException{
		ba.setDroneLocator(false);
		dc.toggleCamera();
		Log.writeLog("KAMERA: Skifter til nedadrettet kamera");			
		Thread.sleep(2000);
		ba.getColorSquares(); // Nulstil listen med objekter
		ba.setObjTrack(true);
		//		ba.setOpticalFlow(false);
	}

	/**
	 * Roterer dronen om sig selv for at finde papkassen. Returnerer distancen til papkassen
	 * hvis den bliver fundet. Returnerer -1 hvis papkassen ikke lokaliseres.
	 * Dronen drejes tilbage til YAW = 0 (bør derefter korrigeres)
	 * @return
	 * @throws InterruptedException
	 */
	private int[] findPapkasse() throws InterruptedException{
		int dist = -1;
		int targetYaw = -45;
		int turns = 1;
		ba.setPapKasseLocator(true);
		Thread.sleep(2000);
		while((dist = ba.getPapKasse()[0]) < 0 && turns < 8){
			ba.setPapKasseLocator(false);
			if(targetYaw < -179){
				targetYaw = 360 + targetYaw;
			}
			dc.turnDroneTo(targetYaw*turns);
			Thread.sleep(1000);
			ba.setPapKasseLocator(true);
			Thread.sleep(3000);
			turns++;
		}
		ba.setPapKasseLocator(false);
		dc.turnDroneTo(0);
		return ba.getPapKasse();
	}

	/**
	 * Udfører en opgaveløsning ved at dreje dronen så lidt som muligt.
	 * Der benyttes primært strafe, fremad og bagud og bevæges i et mønster defineret af dronens position
	 * @throws InterruptedException
	 */
	private void opgave2() throws InterruptedException{
		// Start QR scanning
		ba.setDroneLocator(true);

		// Find droneposition (se fremad)
		Koordinat drone = findDronePos3(); 
		if(drone==null){
			Log.writeLog("*** Start position er IKKE fundet. Programmet afsluttes.");
			return;
		} else {
			dh.turnDroneByPosition(drone);
		}
		// Noter hvor vi er startet
		landingsPlads = drone;
		// Juster position i forhold til hvor meget dronen har bevæget sig for at finde sin første position
		landingsPlads.setX(landingsPlads.getX() + xOffset); 
		landingsPlads.setY(landingsPlads.getY() + yOffset); 
		Log.writeLog("LandingsPlads: " + landingsPlads);
		ba.setDroneLocator(false);
		

		// Lokaliser papkassen i lokalet
//		int[] dist = this.findPapkasse();
//		if(dist[0] > 0){
//			Koordinat papkasse = this.beregnPapkasse(drone, dist[0], dist[1]);
//			// Logisk tjek for om papkassen er nogenlunde i midten af rummet
//			if (papkasse.getX() < 963 && papkasse.getX() > 0 && papkasse.getY() > 0 && papkasse.getY() < 1078){
//				dh.setPapKasse(papkasse);
//				opgrum.setObstacleCenter(papkasse); // Tilføj papkasse på GUI
//				Log.writeLog("Papkasse fundet: " + papkasse);
//			}
//		}

		dh.turnDroneByPosition(drone);
		ba.setDroneLocator(true);
		drone = this.findDronePos3(); // Juster YAW
		dh.turnDroneByPosition(drone);

		int moves = 0; // TODO - Bruges til debug
		DIRECTION lastDir = null;
		// ** START WHILE LØKKE **
		Log.writeLog("*** Starter objektsøgning ***");
		while(dh.getCenterAreal() > 10000){ // Svarer til et område på 2 x 1 meter
			//					while(moves < 8){ // TODO - Bruges til debug

			this.setDroneHeight(235, 250); // Flyv til ca. 2,4 - 2,6 meters højde
			this.modeSwitchObject(); // Start ObjektSøgning

			Thread.sleep(2000); // kig efter objekter i 2 sekunder

			this.modeSwitchQr(); // Stop objektsøgning. Start QR-læsning

			Thread.sleep(200); // Sikrer at billedanalysen ikke er ved at finde yderligere objekter

			// Find objekter hvor vi er nu
			try{
				this.logSquares(drone);
			} catch (Exception e){
				Log.writeLog(e.getMessage());
			}

			this.setDroneHeight(135, 160); // Flyv til QR-kode højde
			
			Koordinat temp = this.findDronePos3();
			if(temp!=null){
				drone = temp;
			}
			dh.turnDroneByPosition(drone);

			// Flyv 1. step baseret på position og retning
			lastDir = dh.moveDrone(drone, lastDir);

			// Estimer dronens nye position
			int xKoor = drone.getX();
			int yKoor = drone.getY();
			switch(lastDir){
			case UP:
				yKoor += 75;
				break;
			case DOWN:
				yKoor -= 75;
				break;
			case LEFT:
				xKoor -= 75;
				break;
			case RIGHT:
				xKoor += 75;
				break;
			case STOPPED:
				break;
			}
			Thread.sleep(2000);
			Koordinat newDronePos = new Koordinat(xKoor, yKoor); // Estimeret position
			lastDir = dh.moveDrone(newDronePos, lastDir);

			// Flyv 2. step baseret på position og retning
			xKoor = newDronePos.getX();
			yKoor = newDronePos.getY();
			switch(lastDir){
			case UP:
				yKoor += 75;
				break;
			case DOWN:
				yKoor -= 75;
				break;
			case LEFT:
				xKoor -= 75;
				break;
			case RIGHT:
				xKoor += 75;
				break;
			case STOPPED:
				break;
			}
			newDronePos = new Koordinat(xKoor, yKoor); // Estimeret position
			
			//			// Find papKasse - vi kan kun se den når vi bevæger os op af y-aksen og kigger mod vinduet
			//			if(lastDir.equals(DIRECTION.UP) && dh.getPapkasse()==null){
			//				ba.setPapKasseLocator(true);
			//			} 
			
			// Find droneposition (se fremad)
			drone = this.findDronePos3();
			// Stop papkasse-søgning
			//			ba.setPapKasseLocator(false);
			if(drone == null){
				drone = newDronePos;
				Log.writeLog("*** Drone position IKKE fundet. Estimerer position: " + drone);
			} 

			// Drej dronen afhængig af positionen så vi peger på den rigtige væg
			boolean adjusted = dh.turnDroneByPosition(drone); 
			if(adjusted){
				drone = this.findDronePos3();
				if(drone==null){
					drone = newDronePos;
					opgrum.setDronePosition(drone, -1*Math.toRadians(dc.getFlightData()[2])); // Opdater GUI med estimeret position
				} else {
					dh.turnDroneByPosition(drone); // YAW er justeret, så vi justerer dronen igen
				}
			}

			//			int dist;
			//			if(dh.getPapkasse()== null && (dist = ba.getPapKasse()) != -1){// Papkassen er fundet
			//				Koordinat papkasse = this.findPapkasse(drone, dist);
			//				// Logisk tjek for om papkassen er nogenlunde i midten af rummet
			//				if (papkasse.getX() < 400 || papkasse.getX() > 800 || papkasse.getY() < 200 || papkasse.getY() > 700){
			//					dh.setPapKasse(papkasse);
			//					opgrum.setObstacleCenter(papkasse); // Tilføj papkasse på GUI
			//				}
			//			}
			moves++; // TODO - Bruges til debug
		}// ** SLUT WHILE LØKKE **

		dc.turnDroneTo(0); // Drej dronen mod vinduesvæggen
		Koordinat temp = drone; // Senest kendte position
		drone = findDronePos3();
		if (drone == null){
			drone = temp;
		} else {
			dc.turnDroneTo(0); // Yaw er justeret, d
		}

		// Find tilbage til landingsplads vha. frem/tilbage/strafe
		dh.flyToKoordinat(drone, landingsPlads);
		Koordinat newDrone = this.findDronePos3(); // Find dronens nye position
		if(newDrone!=null){
			if(newDrone.dist(landingsPlads) > 100){ // Juster positionen i forhold til landingspladsen
				dh.flyToKoordinat(newDrone, landingsPlads);
			}
		}

		Log.writeLog("Opgaveløsning er afsluttet. Dronen lander.");
		return;
	}

	/**
	 * 
	 * @throws InterruptedException
	 */
	private Koordinat beregnPapkasse(Koordinat dronePos, int dist, int x) throws InterruptedException{		
		Koordinat papKasse = new Koordinat(0, 0);
		//Udregn papkassens position i forhold til dronens position og afstand
		int yaw = dc.getFlightData()[2]; //vinkel mellem QR kode og papkasse
		// enhedsvektor = ex = cos(yaw), ey = sin(yaw)
		papKasse.setX((int) Math.cos(yaw+punktNav.getAngle(640, x)) * dist); // vektor x = ex * dist
		papKasse.setY((int) Math.sin(yaw+punktNav.getAngle(640, x)) * dist); // vektor y = ey * dist
		papKasse.setX(papKasse.getX() + dronePos.getX());
		papKasse.setY(papKasse.getY() + dronePos.getY());
		Log.writeLog("Papkassens koordinater er: " + papKasse);
		return papKasse;
	}

	/**
	 * Finder drone position kun ved at kigge fremad. Dronen roteres IKKE.
	 * @return
	 * @throws InterruptedException
	 */
	private Koordinat findDronePos3() throws InterruptedException{
		setDroneHeight(130,160); // Juster dronens højde til QR-koderne

		Thread.sleep(3500); // Vent på stabilt billede

		long start = System.currentTimeMillis();
		Koordinat drone;
		Log.writeLog("Finder drone position...");

		// Kør løkken indtil der er fundet en position. Max 10000ms (10 sek)
		while((drone = ba.getDroneKoordinat()) == null && (System.currentTimeMillis() - start) < 10000){
			if(this.landingsPlads==null){ // Vi -skal- finde vores position hvis vi lige er lettet
				QrFirkant firkanten = ba.getFirkant();
				if(firkanten==null){
					Log.writeLog("Ingen firkant fundet. Flyver baglæns.");
					dc.backward();
					xOffset = xOffset - 100;
					Thread.sleep(3000);
					continue;
				}

				// Find firkant og bevæg dronen mod/væk fra den afhængig af afstand
				if((System.currentTimeMillis() - start) > 3000){ 
					if(firkanten.getHeight() > 500){ // TODO - korrekte pixelværdier for grænsetilfælde
						// bagud
						Log.writeLog("Justerer drone position i forhold til QR-kode. BAGLÆNS");
						dc.backward();
						xOffset = xOffset - 75;
					} else if (firkanten.getHeight() < 100){ // TODO - korrekte pixelværdier for grænsetilfælde
						// fremad
						Log.writeLog("Justerer drone position i forhold til QR-kode. FORLÆNS");
						dc.forward();
						xOffset = xOffset + 75;
					}
					if (firkanten.getCentrum().getX() > 950){ // TODO - korrekte pixelværdier for grænsetilfælde
						// strafe højre
						Log.writeLog("Justerer drone position i forhold til QR-kode. HØJRE");
						dc.strafeRight(0);;
						yOffset = yOffset - 75;
					} else if (firkanten.getCentrum().getX() < 250){ // TODO - korrekte pixelværdier for grænsetilfælde
						// strafe venstre
						Log.writeLog("Justerer drone position i forhold til QR-kode. VENSTRE");
						dc.strafeLeft(0);
						yOffset = yOffset + 75;
					}
					dc.hover();
					Thread.sleep(3000); // Vent på billedet har opdateret og stabiliseret sig
				} else {
					Log.writeLog("DronePosition ikke fundet. Venter på stabilt billede.");
				}
			}
			Thread.sleep(1000);
		}

		if(drone!=null){
			Log.writeLog("Drone position fundet: " + drone);
		}
		//		if(drone!=null && Math.abs(dc.getFlightData()[2]) > 5){ // Drej dronen mod vinduet. YAW er netop blevet opdateret
		//			Log.writeLog("Drejer dronen mod YAW = 0");
		//			dc.turnDroneTo(0);
		//		}
		return drone;
	}

	private void testFlight() throws InterruptedException{
		dc.hover();
		Thread.sleep(3500);
		//		Log.writeLog("Flyver op til 2,5meter");
		//		this.setDroneHeight(240, 260);
		//		Thread.sleep(2000);
		//		Log.writeLog("Flyver ned til QR-højde");
//		this.setDroneHeight(130, 150);
		Thread.sleep(3500);
		
		dh.turnDroneByPosition(new Koordinat(600, 200)); // 90
		Thread.sleep(3000);
		
		dh.turnDroneByPosition(new Koordinat(600, 900)); // -90
		Thread.sleep(3000);
		
		dh.turnDroneByPosition(new Koordinat(100, 500)); // -179/179
		Thread.sleep(3000);
		
		dh.turnDroneByPosition(new Koordinat(863, 200)); // 0
		Thread.sleep(3000);

//		ba.setDroneLocator(true);
//		Koordinat start = this.findDronePos3();
//		if(start!= null){
//			Log.writeLog("Landingsplads: " + start);			
//		}
//		ba.setDroneLocator(false);
//		Thread.sleep(1000);
//
//		ba.setPapKasseLocator(true);
//		dc.turnDroneTo(-135);
//		Thread.sleep(3500);
//
//		int dist;
//		if((dist = ba.getPapKasse()[0]) > 0){
//			Log.writeLog("Distance til papkasse: " + dist + "cm. YAW: " + dc.getFlightData()[2]);
//		}
//
//		dc.turnDroneTo(135);
//		Thread.sleep(3500);
//		if((dist = ba.getPapKasse()[0]) > 0){
//			Log.writeLog("Distance til papkasse: " + dist + "cm. YAW: " + dc.getFlightData()[2]);
//		}
//		ba.setPapKasseLocator(false);
//		Thread.sleep(3500);

		//		dc.forward();
		//		Thread.sleep(3500);
		//		dc.backward();
		//		Thread.sleep(3500);
		//		dc.strafeLeft(0);
		//		Thread.sleep(3500);
		//		dc.strafeRight(0);
		//		Thread.sleep(3500);

//		dc.turnDroneTo(0);
//		Thread.sleep(3500);

		//		dh.flyToKoordinat(new Koordinat(0,0), new Koordinat(125,125));
		//		Thread.sleep(3500);
		destroy();
	}

	//	private Koordinat findDronePos2(){
	//		// Find højden på firkanten rundt om QR koden
	//		BufferedImage bufFrame = dc.getbufImg();
	//		Mat frame = ba.getMatFrame(); // ba.bufferedImageToMat(bufFrame);
	//		ArrayList<QrFirkant> qrFirkanter = punktNav.findQR(frame);
	//
	//		// TODO Læs QR kode og sammenhold position med qrFirkanter objekter
	//		String qrText = qrcs.imageUpdated(bufFrame);
	//
	//		QrFirkant readQr = qrFirkanter.get(0); // TODO
	//		readQr.setText(qrText);
	//		Vector2 v = this.opgrum.getMultiMarkings(qrText)[1];
	//		readQr.setPlacering(new Koordinat((int) v.x, (int) v.y));
	//
	//		// Beregn distancen til QR koden
	//		double dist = punktNav.calcDist(readQr.getHeight(), 420);
	//
	//		// Find vinklen til QR koden
	//		// Dronens YAW + vinklen i kameraet til QR-koden
	//		int yaw = dc.getFlightData()[2];
	//		int imgAngle = (int) punktNav.getAngle(readQr.getCentrum().getX(), bufFrame.getWidth()/2); // DeltaX fra centrum af billedet til centrum af QR koden/firkanten
	//		int totalAngle = yaw + imgAngle;
	//
	//		Koordinat qrPlacering = readQr.getPlacering();
	//		// Beregn dronens koordinat
	//		Koordinat dp = new Koordinat((int) (dist*Math.sin(totalAngle)), (int) (dist*Math.cos(totalAngle)));
	//		dp.setX(qrPlacering.getX() - dp.getX()); //Forskyder i forhold til QR-kodens rigtige markering
	//		dp.setY(qrPlacering.getX() - dp.getX());
	//		System.err.println("DroneKoordinat: (" + dp.getX() + "," + dp.getY() + ")");
	//		System.err.println(qrText);
	//		//		this.opgrum.addGenstandTilKoordinat(dp, new Genstand(COLOR.RØD));
	//		return dp;
	//	}

	//	/**
	//	 * Afsøg rummet indtil der findes en vægmarkering
	//	 * @throws InterruptedException 
	//	 */
	//	private boolean findMark() throws InterruptedException {
	//		if(OPGAVE_DEBUG){
	//			System.err.println("Målsøgning startes.");
	//		}
	//		Log.writeLog("** Målsøgning startes.");
	//		int yaw = 0;
	//		int degrees = 15;
	//		int turns = 0;
	//		long targetStartTime = System.currentTimeMillis();
	//		while((System.currentTimeMillis() - targetStartTime) < searchTime){ // Der søges i max 30 sek
	//
	//			if(Thread.interrupted()){
	//				destroy();
	//				return false;
	//			}
	//			dc.setLedAnim(LEDAnimation.BLINK_ORANGE, 3, 5); // Blink dronens lys orange mens der søges
	//			yaw = dc.getFlightData()[2];
	//			Log.writeLog("Yaw er: " + yaw);
	//			while(Math.abs(yaw - dc.getFlightData()[2]) < degrees){ // drej x grader, søg efter targets
	//				if(Thread.interrupted()){
	//					destroy();
	//					return false;
	//				}
	//				if(OPGAVE_DEBUG){
	//					System.err.println("Intet mål fundet. Drejer dronen.");
	//				}
	//				//				getPossibleManeuvers();
	//				dc.turnLeft();	
	//				Log.writeLog("DREJER VENSTRE");
	//			}
	//			turns++;
	//			if(turns > 250/degrees && (Math.abs(yaw - dc.getFlightData()[2]) < 30)){ // Hvis der er drejet tæt på en fuld omgang, så flyves til nyt sted og søges på ny
	//				if(OPGAVE_DEBUG){
	//					System.err.println("* Intet mål fundet. Dronen skal flyttes.");
	//				}
	//				Log.writeLog("Intet mål fundet. Dronen skal flyttes.");
	//				//Her skal metode implementeres til at flytte dronen til et nyt koordinat i forhold til sidste fundne vægmarkering
	//				//Det antages at dronen ved hvor den er.
	//				if (dronePunkt != null) {
	//					newLocation(dronePunkt);					
	//				}
	//
	//			}
	//			if(qrcs.getQrt() != ""){
	//				Log.writeLog("**Vægmarkering " + qrcs.getQrt() +" fundet.");
	//
	//				Vector2 punkter[] = opgrum.getMultiMarkings(qrcs.getQrt());
	//
	//				//Metode til at udregne vinkel imellem to punkter og dronen skal tilføjes her
	//
	//				//				dronePunkt = pn.udregnDronePunkt(punkter[0], punkter[1], punkter[2], alpha, beta);
	//				Log.writeLog("Dronepunkt "+ dronePunkt.x +  " , " + dronePunkt.y +"fundet.");
	//
	//				//				objektSøgning();
	//
	//				return true;
	//			}
	//			if(OPGAVE_DEBUG){
	//				System.err.println("Intet mål fundet indenfor tidsinterval. Dronen lander.");			
	//			}
	//			Log.writeLog("** Målsøgning afsluttes. Mål IKKE fundet.");
	//			return false;
	//		}
	//		return false;
	//	}


	/**
	 * Hvis opgaven afbrydes af brugeren kaldes denne metode. Dronen lander øjeblikkeligt.
	 */
	private void destroy() {
		doStop = true;
		flying = true;
		System.err.println("*** SKYNET DESTROYED");
		Log.writeLog("*** OpgaveAlgoritme afsluttes. Dronen forsøger at lande.");
		try {			
			dc.land();
			dc.setTimeMode(false);
		} catch (NullPointerException e){
			if(OPGAVE_DEBUG)
				System.err.println("OpgaveAlgoritme.dc er null. Der er ikke forbindelse til dronen.");
		}
		return;
	}


	/**
	 * Flyver dronen til en højde indenfor et interval
	 * @param min Minimumshøjden (i cm)
	 * @param max Maximumshøjden (i cm)
	 * @throws InterruptedException
	 */
	private void setDroneHeight(int min, int max) throws InterruptedException{
		int height = dc.getFlightData()[3];
		while(height < min || height > max ){ 
//			Log.writeLog("Justerer drone højde: " + height);
			if(height < min){ 
				dc.up2();
			} else {
				dc.down2();
			}
			Thread.sleep(300);
			height = dc.getFlightData()[3];
		}
	}

	@Override
	public void run() {
		try {
			this.startOpgaveAlgoritme();	
		} catch (InterruptedException e) {
			this.destroy();
			return;
		}		
	}

	/**
	 * Henter de identificerede objekter. Omregner deres position i billedet til position i rummet
	 * og tilføjer dem til GUI rummet.
	 * @param drone Dronens position ved scanningstidspunktet
	 */
	private void logSquares(Koordinat drone){
		ArrayList<Squares> squares = ba.getColorSquares();
		int height = dc.getFlightData()[3];
		if(squares!=null && !squares.isEmpty()){
			//			Log.writeLog("Fundet " + squares.size() + " genstande inden frasortering.");// TODO - DEBUG
			for(Squares sq : squares){
				//				Log.writeLog("FOR-løkke startet.");// TODO - DEBUG
				Genstand g;
				if(sq.getFarve().equals(FARVE.GR�N)){
					g = new Genstand(GENSTAND_FARVE.GR�N);
				} else {
					g = new Genstand(GENSTAND_FARVE.R�D);
				}	
				//				Log.writeLog("Genstand objekt lavet.");// TODO - DEBUG
				Koordinat k = opgrum.rotateCoordinate(sq, drone, height);
				//				Log.writeLog("RotateCoordinate udført. Drone Koordinat: " + drone);// TODO - DEBUG
				opgrum.addGenstandTilKoordinat(k, g);
				//				Log.writeLog("Genstand tilføjet til OpgaveRum: " +k + "\tBilledKoordinat: (" + sq.x + "," + sq.y + ")"); // TODO - DEBUG
			}
		} else {
			Log.writeLog("Ingen objekter fundet! " + drone);
		}
	}
}
