resourcestring                                     {english}

//  gmapURL='https://maps.google.com/maps';
   osmURL='https://www.openstreetmap.org/';         {als Karte}
//  osmURL='https://opentopomap.org/';             {als topografische Karte}
//  https://opentopomap.org/#map=17/48.92523/14.26150

  capSatSNR='Signal to noise ratio';
  hntSatSNR='Signal to noise ratio of satellites  [dB]';
  capData='Positioning data';
  hntData='Positioning data from message GPS_RAW_INT';
  capPolar='Azimuth and elevation of visible sats';
  hntPolar='Azimuth: Direction of satellite 0 to 360°, Elevation: 0 to 90° (0°: right on top of receiver, 90°: on the horizon)';
  hntClose='Disconnect and quit';
  hntStatusBar='Number messages sent | Number messages received | Status';
  capTime='Time';
  hntTime='Date/time UTC from message SYSTEM_TIME (2)';
  capSaveCSV='Save as CSV file';
  hntSaveCSV='Automatically save GPS data as CSV file in the same folder as the input files are';
  capFast='Fast processing w/o GUI update';
  hntFast='Fast processing without update GUI but creates CSV data. Automatic save should be checked';
  hntSleepTime='Set delay in ms (0..1000), default: 100';

  capRecord='Record received data messages';
  hntRecord='Record the received data messages to a CSV file in hexa-decimal format';
  capSensor='Create sensor file';
  hntSensor='Save received data messages to a binary file like Sensor files in flight logs';
  capSaveProt='&Save text';
  hntSaveProt='Save list of text messages to text file';
  hntPort='In case of connection Problems unplug and plug USB cable.'+lineending+'Double click to update port list.';

  capSaveChart='Save chart';
  hntSaveChart='Save snapshot of the diagram as picture';
  capAbout='About';
  capFCtime='Time since boot';
  hntFCtime='Time since boot of flight controller (since drone was powerd up)';

  capMainMenuFile='File';
  capMainMenuInfo='Info';

  errInvalidFile='Invalid file';
  errSmallFile='File too small';
  errEmptyFile='File is empty, nothing saved';
  errSensorsNotHealthy='Sensors not healthy!';

  rsInUse='satellites in use';
  rsVisible='satellites visible';
  rsSaved=' saved';
  rsDisconnected='Disconnected';


  hntGeoFence='Maximum distance from home point in Smart Mode.';
  hntHeightLimit='Maximum elevation above home point in Smart and Angle mode.';

  msgPropellerRemoved='I have removed all propellers.';
  hntEnableTesting='Test motors only without propellers!';


