        {********************************************************}
        {                                                        }
        {       Read and send data to Yuneec CGO3+ camera        }
        {                                                        }
        {       Copyright (c) 2025         Helmut Elsner         }
        {                                                        }
        {       Compiler: FPC 3.2.3   /    Lazarus 3.7           }
        {                                                        }
        { Pascal programmers tend to plan ahead, they think      }
        { before they type. We type a lot because of Pascal      }
        { verboseness, but usually our code is right from the    }
        { start. We end up typing less because we fix less bugs. }
        {           [Jorge Aldo G. de F. Junior]                 }
        {********************************************************}

(*
This source is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free
Software Foundation; either version 2 of the License, or (at your option)
any later version.

This code is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
details.

A copy of the GNU General Public License is available on the World Wide Web
at <http://www.gnu.org/copyleft/gpl.html>. You can also obtain it by writing
to the Free Software Foundation, Inc., 51 Franklin Street - Fifth Floor,
Boston, MA 02110-1335, USA.


THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*******************************************************************************)

{This unit needs following additional components:
- Synapse
- Industrial stuff

Also the units mav_def and mav_msg from repository "Common units" are needed:
https://github.com/h-elsner/common_units
The unit msg57 is a dummy for Yuneec NFZ license procedure,
which is not open source.
}

unit CGO3read_main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, ExtCtrls, StdCtrls,
  lclintf, lcltype, Buttons, ActnList, Menus, Process, XMLPropStorage, ComCtrls, Grids,
  ValEdit, Spin, TAGraph, TATypes, TASeries, TAChartUtils, TAGeometry,
  TARadialSeries, TASources, TAIntervalSources, TATools, synaser, MKnob,
  clipbrd, mav_def, mav_msg, msg57, Types;

type

  { TForm1 }

  TForm1 = class(TForm)
    acConnect: TAction;
    acClose: TAction;
    acDisconnect: TAction;
    acScanPorts: TAction;
    acSaveGUItext: TAction;
    acCopySerial: TAction;
    acEnableTesting: TAction;
    acScreenshot: TAction;
    ActionList1: TActionList;
    btnScreenshot: TBitBtn;
    btnEnableNFZ: TButton;
    btnGimbalCali: TButton;
    btnSaveMsg: TBitBtn;
    btnVersion: TButton;
    btnZeroPhaseCali: TButton;
    btnYawEncCali: TButton;
    btnAccCali: TButton;
    btnZeroPhaseErs: TButton;
    btnAccErase: TButton;
    btnYawEncErs: TButton;
    btnPreFrontCali: TButton;
    btnDisconnect: TBitBtn;
    btnClose: TBitBtn;
    btnConnect: TBitBtn;
    btnCenter: TButton;
    btnFrontCali: TButton;
    btnFrontErs: TButton;
    btnGeoFence: TButton;
    btnHeightLimit: TButton;
    btnDisableNFZ: TButton;
    btnEnableTesting: TButton;
    btnTurnAll: TButton;
    cbPort: TComboBox;
    cbRecord: TCheckBox;
    cbSensor: TCheckBox;
    cbSpeed: TComboBox;
    cbTelemetry: TCheckBox;
    cbLimitMsg: TCheckBox;
    cbHighRPM: TCheckBox;
    chAddValue: TChart;
    AddValueLineSeries1: TLineSeries;
    DateTimeIntervalChartSource1: TDateTimeIntervalChartSource;
    gbMag: TGroupBox;
    gbOrientation: TGroupBox;
    gbBaro: TGroupBox;
    gbSysStatus: TGroupBox;
    gbAcc: TGroupBox;
    gbGyro: TGroupBox;
    gbMotors: TGroupBox;
    Image1: TImage;
    lblOtherSats: TLabel;
    mnEnableTesting: TMenuItem;
    mnCopySerial: TMenuItem;
    mnSaveGUItext: TMenuItem;
    rgAddValue: TRadioGroup;
    Separator1: TMenuItem;
    picGPS: TImage;
    picGLONASS: TImage;
    picSBAS: TImage;
    ImageList1: TImageList;
    lbIGPSsats: TLabel;
    lbIGLONASSsats: TLabel;
    lbISBASsats: TLabel;
    lbIIESCOK: TLabel;
    picMotors: TImage;
    lblEnableTesting: TLabel;
    lblOK: TLabel;
    lblFCtime: TLabel;
    lblFCtimeGPS: TLabel;
    lbIGPSOK: TLabel;
    lbIIMUOK: TLabel;
    lbIRSOK: TLabel;
    lbISonar: TLabel;
    lblGeoFenceVal: TLabel;
    lblHeightLimit: TLabel;
    lblGeoFence: TLabel;
    lblCurrentValue: TLabel;
    lblHeightLimitVal: TLabel;
    lblNewValue: TLabel;
    lblSatUsed: TLabel;
    lblNotUsed: TLabel;
    GUIpanel: TPanel;
    lblSysTime: TLabel;
    mnLicense: TPopupMenu;
    SensorLEDPanel: TPanel;
    SatPolarSeries: TPolarSeries;
    SatPolarSource: TListChartSource;
    SatPolar: TChart;
    ChartSatSNR: TChart;
    BarSatSNR: TBarSeries;
    chPanLineSeries1: TLineSeries;
    chRoll: TChart;
    chRollLineSeries1: TLineSeries;
    chTilt: TChart;
    chPan: TChart;
    chTiltLineSeries1: TLineSeries;
    gridVarious: TStringGrid;
    gbDeviceInfo: TGroupBox;
    gbGeoFence: TGroupBox;
    gbPosition: TGroupBox;
    gbVelocity: TGroupBox;
    gbMessages: TGroupBox;
    knPanControl: TmKnob;
    lblWarning: TLabel;
    lblSerial: TLabel;
    lblSerialNo: TLabel;
    lblPowerCycle: TLabel;
    lblGimbalBootTime: TLabel;
    lblBootTime: TLabel;
    lblGimbalVersion: TLabel;
    lblPanControl: TLabel;
    GIMBALtext: TMemo;
    GUItext: TMemo;
    SatSNRBarSource: TListChartSource;
    pcMain: TPageControl;
    pcGUI: TPageControl;
    panelRight: TPanel;
    panelYGCTop: TPanel;
    rgYGC_Type: TRadioGroup;
    rgPanMode: TRadioGroup;
    rgTiltMode: TRadioGroup;
    SaveDialog1: TSaveDialog;
    shapeGPSOK: TShape;
    shapeIMUOK: TShape;
    shapeESCOK: TShape;
    shapeNotUsed: TShape;
    shapeRSOK: TShape;
    shapeSonar: TShape;
    shapeUsed: TShape;
    speGeoFence: TSpinEdit;
    speHeightLimit: TSpinEdit;
    StatusBar1: TStatusBar;
    gridStatus: TStringGrid;
    timerSensors: TTimer;
    tsSettings: TTabSheet;
    tsGPSinfo: TTabSheet;
    tsSensorInfo: TTabSheet;
    timerGUI: TTimer;
    tsGUI: TTabSheet;
    timerYGCcommandLong: TTimer;
    timerFCCommand: TTimer;
    timerTelemetry: TTimer;
    timerFCHeartbeat: TTimer;
    tbTiltControl: TTrackBar;
    tsFC: TTabSheet;
    tsYGC: TTabSheet;
    upperPanel: TPanel;
    vleMag: TValueListEditor;
    vleOrientation: TValueListEditor;
    vleBaro: TValueListEditor;
    vlePosition: TValueListEditor;
    vleSystem: TValueListEditor;
    vleVelocity: TValueListEditor;
    vleSysStatus: TValueListEditor;
    vleAcc: TValueListEditor;
    vleGyro: TValueListEditor;
    XMLPropStorage1: TXMLPropStorage;

    procedure acCloseExecute(Sender: TObject);
    procedure acConnectExecute(Sender: TObject);
    procedure acCopySerialExecute(Sender: TObject);
    procedure acDisconnectExecute(Sender: TObject);
    procedure acEnableTestingExecute(Sender: TObject);
    procedure acSaveGUItextExecute(Sender: TObject);
    procedure acScanPortsExecute(Sender: TObject);
    procedure acScreenshotExecute(Sender: TObject);
    procedure btnAccCaliClick(Sender: TObject);
    procedure btnAccEraseClick(Sender: TObject);
    procedure btnCenterClick(Sender: TObject);
    procedure btnDisableNFZClick(Sender: TObject);
    procedure btnEnableNFZClick(Sender: TObject);
    procedure btnFrontErsClick(Sender: TObject);
    procedure btnGeoFenceClick(Sender: TObject);
    procedure btnHeightLimitClick(Sender: TObject);
    procedure btnPreFrontCaliClick(Sender: TObject);
    procedure btnTurnAllClick(Sender: TObject);
    procedure btnVersionClick(Sender: TObject);
    procedure btnYawEncCaliClick(Sender: TObject);
    procedure btnYawEncErsClick(Sender: TObject);
    procedure btnZeroPhaseCaliClick(Sender: TObject);
    procedure btnZeroPhaseErsClick(Sender: TObject);
    procedure btnGimbalCaliClick(Sender: TObject);
    procedure cbPortDblClick(Sender: TObject);
    procedure FormActivate(Sender: TObject);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure GIMBALtextMouseWheelDown(Sender: TObject; Shift: TShiftState;
      MousePos: TPoint; var Handled: Boolean);
    procedure GIMBALtextMouseWheelUp(Sender: TObject; Shift: TShiftState;
      MousePos: TPoint; var Handled: Boolean);
    procedure GUItextMouseWheelDown(Sender: TObject; Shift: TShiftState;
      MousePos: TPoint; var Handled: Boolean);
    procedure GUItextMouseWheelUp(Sender: TObject; Shift: TShiftState;
      MousePos: TPoint; var Handled: Boolean);
    procedure Image1Click(Sender: TObject);
    procedure knPanControlChange(Sender: TObject; AValue: Longint);
    procedure picMotorsMouseDown(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure rgAddValueClick(Sender: TObject);
    procedure rgYGC_TypeClick(Sender: TObject);
    procedure SatPolarAfterDrawBackWall(ASender: TChart; ACanvas: TCanvas;
      const ARect: TRect);
    procedure timerFCCommandTimer(Sender: TObject);
    procedure timerFCHeartbeatTimer(Sender: TObject);
    procedure timerGUITimer(Sender: TObject);
    procedure timerSensorsTimer(Sender: TObject);
    procedure timerTelemetryTimer(Sender: TObject);
    procedure timerYGCcommandLongTimer(Sender: TObject);
    procedure vleAccPrepareCanvas(Sender: TObject; aCol, aRow: Integer;
      aState: TGridDrawState);

  private
    procedure StopAllTimer;
    procedure ResetSensorsStatus;
    procedure CreateFCControl(var msg: TMavMessage; SequenceNumber: byte; const command: uint16=$FFFF);
    procedure CreateFCTelemetry5GHz(var msg: TMavMessage; SequenceNumber: byte);
    function PanModeToInt: uint16;
    function TiltModeToInt: uint16;

    procedure GridPrepare(var grid: TStringGrid; const NumRows: byte);
    procedure WriteHeader_STATUS;
    procedure WriteHeader_GYRO_POWER;
    procedure WriteHeader_EULER_ANGLE;
    procedure WriteHeader_ACC;
    procedure WriteHeader_TEMP_DIFF;
    procedure WriteHeader_Channel_data;
    procedure WriteGUIvalueListHeader;
    procedure ClearMessageTables;
    procedure ClearGUI;
    procedure FillGUIPosition24(const sats: TGPSdata);
    procedure FillGPS_STATUS(const sats: TGPSdata);
    procedure FillGUIPosition33(const sats: TGPSdata);
    procedure FillGUIIMU(const data: THWstatusData);
    procedure FillGUI_SYS_STATUS(const msg: TMAVmessage; var data: TGPSdata);
    procedure FillSENSOR_OFFSETS(const data: THWstatusData);
    procedure FillCharts;
    procedure FillAttitude(data: TAttitudeData);
    procedure FillEKF_STATUS_REPORT(data: TAttitudeData);

    procedure CreateSatSNRBarChart(const sats: TGPSdata);
    procedure CreateSatPolarDiagram(const sats: TGPSdata);
    procedure PrepareSatSNRBarChart;                   {My settings for the SNR chart}
    procedure PreparePolarAxes(AChart: TChart; AMax: Double);
    procedure PrepareSatPolarDiagram;
    procedure DrawPolarAxes(AChart: TChart; AMax, ADelta: Double; MeasurementUnit: string);
    procedure GUIsetCaptionsAndHints;
    procedure TakeScreenshot(filename: string);
    procedure AddToChart(const Time: TDateTime; value: single);
  public
    procedure ReadMessage_FE(var msg: TMAVmessage);
    procedure ReadGimbalPosition(msg: TMAVmessage);
    procedure ReadMessage_BC(var msg: TMAVmessage);
    procedure ReadGUIMessages(msg: TMAVmessage);
    procedure RecordMessage(msg: TMAVmessage; list: TStringList; LengthFixPart: byte);
    procedure ActAsFlightController(var msg: TMAVmessage; list: TStringList);
    procedure ActAsGimbalChecker(var msg: TMAVmessage; list: TStringList);
    procedure ActAsGUI(var msg: TMAVmessage; list: TStringList);
    procedure ReadYGCcameraMessages(msg: TMAVmessage);
    procedure SendYGCHeartbeat;
    procedure NumberMessagesInStatusBar;

    function YGC_TimestampIn_ms(msg: TMAVmessage): integer;
    procedure GIMBAL_GYRO_POWER(msg: TMAVmessage);
    procedure GIMBAL_EULER_ANGLE(msg: TMAVmessage);
    procedure GIMBAL_ACC(msg: TMAVmessage);
    procedure GIMBAL_TEMP_DIFF(msg: TMAVmessage);
    procedure GIMBAL_STATUS(msg: TMAVmessage);
    procedure Channel_data(msg: TMAVmessage);
    procedure CAM_SERIAL(msg: TMAVmessage);
    procedure TEXT_MESSAGE(msg: TMAVmessage);
  end;

  {$I YTHtool_en.inc}

var
  Form1: TForm1;
  UART: TBlockSerial;
  UARTConnected, SerialNumberFound: boolean;
  SensorStream: TMemoryStream;

  starttime: UInt64;
  boottime: TDateTime;
  SequNumberTransmit, SequNumberReceived: byte;
  MessagesSent, MessagesReceived: integer;
  pan, roll, tilt, voltage: uint16;

const
  AppVersion='V1.5 2024-01-22';
  linkLazarus='https://www.lazarus-ide.org/';

  tab1=' ';
  tab2='  ';

  maxPorts=10;
  timeout=100;
  defaultbaud=115200;
  wait=5;
  highRPM=100;

  AccMin=850;          {Threshold for accelerometer magnitude (z-axis problem}
  gzoom='16';

  clSatUsed=clGreen;
  clSatVisible=$004080FF;
  clPolarLabel=clSkyBlue;
  clDataSerie1=clRed;
  clSensorOK=clMoneyGreen;
  clSensorMiss=$000065FF;
  PolarSatSize=8;

{$IFDEF WINDOWS}
  default_port='COM6';
{$ELSE}                                                {UNIX like OS}
  default_port='/dev/ttyUSB0';
{$ENDIF}


implementation

{$R *.lfm}

{ TForm1 }

procedure TForm1.FormCreate(Sender: TObject);
begin
  UARTconnected:=false;
  GIMBALtext.Text:='';
  GUItext.Text:='';
  GUIsetCaptionsAndHints;

  WriteHeader_STATUS;
  WriteHeader_GYRO_POWER;
  WriteGUIvalueListHeader;


  ResetSensorsStatus;
  PrepareSatSNRBarChart;
  PrepareSatPolarDiagram;
  PreparePolarAxes(SatPolar, 90);
  BarSatSNR.Clear;
  SatPolarSeries.Clear;
  AddValueLineSeries1.Clear;

  shapeUsed.Pen.Color:=clSatUsed;
  shapeNotUsed.Pen.Color:=clSatVisible;
end;

procedure TForm1.GUIsetCaptionsAndHints;
begin
  Caption:=Application.Title+tab2+AppVersion;
  cbSpeed.Text:=IntToStr(defaultbaud);
  cbSpeed.Hint:=hntSpeed;
  cbPort.Hint:=hntPort;
  acConnect.Caption:=capConnect;
  acConnect.Hint:=hntConnect;
  acDisConnect.Caption:=capDisConnect;
  acDisConnect.Hint:=hntDisConnect;
  acClose.Caption:=capClose;
  acSaveGUItext.Caption:=capSaveGUItext;
  acSaveGUItext.Hint:=hntSaveGUItext;
  acCopySerial.Caption:=capCopySerial;
  acCopySerial.Hint:=hntCopySerial;
  acScreenshot.Caption:=capScreenshot;
  acScreenshot.Hint:=hntScreenshot;

  btnGimbalCali.Caption:=capGimbalCali;
  btnGimbalCali.Hint:=hntGimbalCali;
  btnVersion.Caption:=capVersion;
  btnVersion.Hint:=hntVersion;
  btnCenter.Caption:=capCenter;
  btnCenter.Hint:=hntCenter;

  acSaveGUItext.Caption:=capSaveProt;
  acSaveGUItext.Hint:=hntSaveProt;
  acEnabletesting.Caption:=capEnableTesting;

  lblOtherSats.Caption:='';
  lblGeoFenceVal.Hint:=hntGeoFence;
  speGeoFence.Hint:=hntGeoFence;
  lblGeoFence.Hint:=hntGeoFence;
  lblHeightLimitVal.Hint:=hntHeightLimit;
  speHeightLimit.Hint:=hntHeightLimit;
  lblHeightLimit.Hint:=hntHeightLimit;
  lblBootTime.Caption:=capFCtime;
  lblBootTime.Hint:=hntBoottime;
  lblFCtimeGPS.Caption:=capFCtime;
  lblFCtimeGPS.Hint:=hntFCtime;
  lblFCtime.Caption:=capFCtime;
  lblFCtime.Hint:=hntFCtime;
  lbIGPSsats.Hint:=hntGPS;
  lbIGLONASSsats.Hint:=hntGLONASS;
  lbISBASsats.Hint:=hntSBAS;
  lblSystime.Caption:=capSystemTime;
  lblSysTime.Hint:=hntTime;
  lblNotUsed.Caption:=capNotUsed;
  lblNotUsed.Hint:=hntNotUsed;
  lblSatUsed.Caption:=capSatUsed;
  lblSatUsed.Hint:=hntSatUsed;
  lblSerialNo.Caption:=capSerialNo;
  lblEnableTesting.Caption:=hntEnableTesting;

  SatPolar.Hint:=hntPolar;
  ChartSatSNR.Hint:=hntSatSNR;
  btnClose.Hint:=hntClose;
  StatusBar1.Hint:=hntStatusBar;
  rgYGC_Type.Caption:=capYGC_Type;
  rgYGC_Type.Hint:=hntYGC_Type;
  panelRight.Hint:=hntPanelRight;
  tbTiltControl.Hint:=hntTiltControl;
  rgTiltMode.Hint:=hntTiltMode;
  rgPanMode.Hint:=hntPanMode;
  knPanControl.Hint:=hntPanControl;
  gbMotors.Hint:=hntEnableTesting;

  cbLimitMsg.Caption:=capLimitMsg;
  cbHighRPM.Caption:=capHighRPM;
  cbHighRPM.Hint:=hntHighRPM;
  cbRecord.Caption:=capRecord;
  cbRecord.Hint:=hntRecord;
  cbSensor.Caption:=capSensor;
  cbSensor.Hint:=hntSensor;
  cbTelemetry.Caption:=capTelemetry;
  cbTelemetry.Hint:=hntTelemetry;
  chAddValue.Hint:=capAddValue;
  rgAddValue.Caption:=capAddValue;
  rgAddValue.Hint:=hntAddValue;

  tsGUI.Caption:=captsGUI;
  tsYGC.Caption:=captsYGC;
  tsFC.Caption:=captsFC;
  tsSensorInfo.Caption:=capSensorInfo;
  tsGPSinfo.Caption:=capGPSinfo;
  tsSettings.Caption:=capSettings;
end;

procedure TForm1.GIMBALtextMouseWheelDown(Sender: TObject; Shift: TShiftState;
  MousePos: TPoint; var Handled: Boolean);
begin
  if ssCtrl in Shift then
    GIMBALtext.Font.Size:=GIMBALtext.Font.Size-1;
end;

procedure TForm1.GIMBALtextMouseWheelUp(Sender: TObject; Shift: TShiftState;
  MousePos: TPoint; var Handled: Boolean);
begin
  if ssCtrl in Shift then
    GIMBALtext.Font.Size:=GIMBALtext.Font.Size+1;
end;

procedure TForm1.GUItextMouseWheelDown(Sender: TObject; Shift: TShiftState;
  MousePos: TPoint; var Handled: Boolean);
begin
  if ssCtrl in Shift then
    GUItext.Font.Size:=GUItext.Font.Size-1;
end;

procedure TForm1.GUItextMouseWheelUp(Sender: TObject; Shift: TShiftState;
  MousePos: TPoint; var Handled: Boolean);
begin
  if ssCtrl in Shift then
    GUItext.Font.Size:=GUItext.Font.Size+1;
end;

procedure TForm1.Image1Click(Sender: TObject);
begin
  OpenURL(linkLazarus);
end;

function SendUARTMessage(const msg: TMAVmessage; LengthFixPart: byte): boolean;
begin
  result:=false;
  if msg.valid then begin
    if UART.SendBuffer(@msg.msgbytes, msg.msglength+LengthFixPart+2)>LengthFixPart then begin
      result:=true;
      inc(MessagesSent);
    end;
  end;
  sleep(wait);
end;

procedure TForm1.GridPrepare(var grid: TStringGrid; const NumRows: byte);
var
  i: byte;

begin
  grid.RowCount:=NumRows;
  for i:=1 to NumRows-1 do
    grid.Cells[1, i]:='';
end;

procedure TForm1.WriteHeader_STATUS;
begin
  GridPrepare(gridStatus, 15);
  gridStatus.Cells[0, 0]:='STATUS';
  gridStatus.Cells[1, 0]:='Value';
  gridVarious.Cells[1, 0]:='Value';
  gridStatus.Cells[0, 1]:='Voltage';
  gridStatus.Cells[0, 2]:='Ampere';
  gridStatus.Cells[0, 3]:='Seconds';
  gridStatus.Cells[0, 4]:='EncDataP';
  gridStatus.Cells[0, 5]:='EncDataR';
  gridStatus.Cells[0, 6]:='EncDataY';
  gridStatus.Cells[0, 7]:='StageAngleX';
  gridStatus.Cells[0, 8]:='StageAngleY';
  gridStatus.Cells[0, 9]:='AircraftAngleX';
  gridStatus.Cells[0, 10]:='AircraftAngleY';
  gridStatus.Cells[0, 11]:='AircraftAngleZ';
  gridStatus.Cells[0, 12]:='GyroStableX';
  gridStatus.Cells[0, 13]:='GyroStableY';
  gridStatus.Cells[0, 14]:='GyroStableZ';
end;

procedure TForm1.WriteHeader_GYRO_POWER;
begin
  GridPrepare(gridVarious, 16);
  gridVarious.Cells[0, 0]:='GYRO_POWER';
  gridVarious.Cells[0, 1]:='GyroHopeX';
  gridVarious.Cells[0, 2]:='GyroHopeY';
  gridVarious.Cells[0, 3]:='GyroHopeZ';
  gridVarious.Cells[0, 4]:='GyroX';
  gridVarious.Cells[0, 5]:='GyroY';
  gridVarious.Cells[0, 6]:='GyroZ';
  gridVarious.Cells[0, 7]:='SpeedHopeP';
  gridVarious.Cells[0, 8]:='SpeedHopeR';
  gridVarious.Cells[0, 9]:='SpeedHopeY';
  gridVarious.Cells[0, 10]:='SpeedP';
  gridVarious.Cells[0, 11]:='SpeedR';
  gridVarious.Cells[0, 12]:='SpeedY';
  gridVarious.Cells[0, 13]:='PowerP';
  gridVarious.Cells[0, 14]:='PowerR';
  gridVarious.Cells[0, 15]:='PowerY';
end;

procedure TForm1.WriteHeader_EULER_ANGLE;
begin
  GridPrepare(gridVarious, 16);
  gridVarious.Cells[0, 0]:='EULER_ANGLE';
  gridVarious.Cells[0, 1]:='AnglehopeX';
  gridVarious.Cells[0, 2]:='AnglehopeY';
  gridVarious.Cells[0, 3]:='AnglehopeZ';
  gridVarious.Cells[0, 4]:='AngleX';
  gridVarious.Cells[0, 5]:='AngleY';
  gridVarious.Cells[0, 6]:='AngleZ';
  gridVarious.Cells[0, 7]:='EulerHopeP';
  gridVarious.Cells[0, 8]:='EulerHopeR';
  gridVarious.Cells[0, 9]:='EulerHopeY';
  gridVarious.Cells[0, 10]:='EulerP';
  gridVarious.Cells[0, 11]:='EulerR';
  gridVarious.Cells[0, 12]:='EulerY';
  gridVarious.Cells[0, 13]:='AngleErrorX';
  gridVarious.Cells[0, 14]:='AngleErrorY';
  gridVarious.Cells[0, 15]:='AngleErrorZ';
end;

procedure TForm1.WriteHeader_ACC;
begin
  GridPrepare(gridVarious, 16);
  gridVarious.Cells[0, 0]:='ACC';
  gridVarious.Cells[0, 1]:='AccLowPassX';
  gridVarious.Cells[0, 2]:='AccLowPassY';
  gridVarious.Cells[0, 3]:='AccLowPassZ';
  gridVarious.Cells[0, 4]:='AccLowPass';
  gridVarious.Cells[0, 5]:='AccX';
  gridVarious.Cells[0, 6]:='AccY';
  gridVarious.Cells[0, 7]:='AccZ';
  gridVarious.Cells[0, 8]:='Acc';
  gridVarious.Cells[0, 9]:='HorizontalAcc';
  gridVarious.Cells[0, 10]:='VerticalAcc';
  gridVarious.Cells[0, 11]:='AccKp';
  gridVarious.Cells[0, 12]:='AccKi';
  gridVarious.Cells[0, 13]:='AccKd';
  gridVarious.Cells[0, 14]:='HorizontalAccChageX';
  gridVarious.Cells[0, 15]:='HorizontalAccChageY';
end;

procedure TForm1.WriteHeader_TEMP_DIFF;
begin
  GridPrepare(gridVarious, 14);
  gridVarious.Cells[0, 0]:='TEMP_DIFF';
  gridVarious.Cells[0, 1]:='HistoryZeroToleranceX';
  gridVarious.Cells[0, 2]:='HistoryZeroToleranceY';
  gridVarious.Cells[0, 3]:='HistoryZeroToleranceZ';
  gridVarious.Cells[0, 4]:='ZeroToleranceX';
  gridVarious.Cells[0, 5]:='ZeroToleranceY';
  gridVarious.Cells[0, 6]:='ZeroToleranceZ';
  gridVarious.Cells[0, 7]:='ZeroVariationX';
  gridVarious.Cells[0, 8]:='ZeroVariationY';
  gridVarious.Cells[0, 9]:='ZeroVariationZ';
  gridVarious.Cells[0, 10]:='TempHope';
  gridVarious.Cells[0, 11]:='HeatPower';
  gridVarious.Cells[0, 12]:='TempIntegral';
  gridVarious.Cells[0, 13]:='IMUTemp';
end;

procedure TForm1.WriteHeader_Channel_data;
begin
  GridPrepare(gridVarious, 9);
  gridVarious.Cells[0, 0]:='Channel_data';
  gridVarious.Cells[0, 1]:='Tilt mode';
  gridVarious.Cells[0, 2]:='Tilt';
  gridVarious.Cells[0, 3]:='Ch3 unused';
  gridVarious.Cells[0, 4]:='Ch4 reserve';
  gridVarious.Cells[0, 5]:='Pan mode';
  gridVarious.Cells[0, 6]:='Pan knob';
  gridVarious.Cells[0, 7]:='Ch7 unused';
  gridVarious.Cells[0, 8]:='Ch8 unused';
end;

procedure SetXYZ(var vle: TValueListEditor; const MeasurementUnit: shortstring='');
begin
  vle.Cells[0, 1]:='X '+MeasurementUnit;
  vle.Cells[0, 2]:='Y '+MeasurementUnit;
  vle.Cells[0, 3]:='Z '+MeasurementUnit;
end;

procedure TForm1.WriteGUIvalueListHeader;
begin
  vlePosition.Cells[0, 1]:='Latitude';
  vlePosition.Cells[0, 2]:='Longitude';
  vlePosition.Cells[0, 3]:='Altitude MSL';
  vlePosition.Cells[0, 4]:='Altitude rel';
  vlePosition.Cells[0, 5]:='Sats visible';
  vlePosition.Cells[0, 6]:='Sats used';
  vlePosition.Cells[0, 7]:='HDOP';
  vlePosition.Cells[0, 8]:='VDOP';
  vlePosition.Cells[0, 9]:='Fix type';

  vleVelocity.Cells[0, 1]:='Velocity';
  vleVelocity.Cells[0, 2]:='Vx';
  vleVelocity.Cells[0, 3]:='Vy';
  vleVelocity.Cells[0, 4]:='Vz';
  vleVelocity.Cells[0, 5]:='Variance';

  vleSystem.Cells[0, 1]:='Vehicle type';
  vleSystem.Cells[0, 2]:='Vehicle ID';
  vleSystem.Cells[0, 3]:='FW version';
  vleSystem.Cells[0, 4]:='FW date';
  vleSystem.Cells[0, 5]:='Real Sense';

  vleSysStatus.Cells[0, 1]:='Sensors present';
  vleSysStatus.Cells[0, 2]:='Sensors enabled';
  vleSysStatus.Cells[0, 3]:='Sensors healty';
  vleSysStatus.Cells[0, 4]:='Drop rate';
  vleSysStatus.Cells[0, 5]:='Comm erros';
  vleSysStatus.Cells[0, 6]:='Error count';
  vleSysStatus.Cells[0, 7]:='EKF status';
  vleSysStatus.Cells[0, 8]:='Voltage';
  vleSysStatus.Cells[0, 9]:='Radio SR24';

  vleBaro.Cells[0, 1]:='Pressure';
  vleBaro.Cells[0, 2]:='Temperature';
  vleBaro.Cells[0, 3]:='Height estimate';

  SetXYZ(vleGyro);
  vleGyro.Cells[0, 4]:='Gyro cali X';
  vleGyro.Cells[0, 5]:='Gyro cali Y';
  vleGyro.Cells[0, 6]:='Gyro cali Z';
  vleGyro.Cells[0, 7]:='IMU temperature';

  SetXYZ(vleAcc, '[mG]');
  vleAcc.Cells[0, 4]:='Magnitude [mG]';
  vleAcc.Cells[0, 5]:='Acc cali X';
  vleAcc.Cells[0, 6]:='Acc cali Y';
  vleAcc.Cells[0, 7]:='Acc cali Z';

  SetXYZ(vleMag);
  vleMag.Cells[0, 4]:='Compass variance';
  vleMag.Cells[0, 5]:='Mag offset X';
  vleMag.Cells[0, 6]:='Mag offset Y';
  vleMag.Cells[0, 7]:='Mag offset Z';

  vleOrientation.Cells[0, 1]:='Roll';
  vleOrientation.Cells[0, 2]:='Pitch';
  vleOrientation.Cells[0, 3]:='Yaw';

  ResetSensorsStatus;
end;

procedure TForm1.ClearMessageTables;
var
  i: integer;

begin
  for i:=1 to gridStatus.RowCount-1 do
    gridStatus.Cells[1, i];
  for i:=1 to gridVarious.RowCount-1 do
    gridVarious.Cells[1, i];
end;

function GetContrastTextColor(const BackColor: TColor): TColor;
begin                                              {Textfaebe abh. vom Hintergrund}
  if (Red(BackColor) * 0.25+
      Green(BackColor) * 0.625+
      Blue(BackColor) * 0.125) > 90 then
    result := clBlack
  else
    result := clWhite;
end;

procedure CellColorSetting(aGrid: TValueListEditor; Farbe: TColor); {Zellen einfärben}
begin
  aGrid.Canvas.Brush.Color:=Farbe;
  aGrid.Canvas.Font.Color:=GetContrastTextColor(Farbe);
end;

function InvertPanControlPosition(pos: uint16): uint16;
begin
  result:=4096-pos;
end;

function TForm1.TiltModeToInt: uint16;
begin
  case rgTiltMode.ItemIndex of
    0: result:=2184;
    1: result:=3412;
  end;
end;

function TForm1.PanModeToInt: uint16;
begin
  case rgPanMode.ItemIndex of
    0: result:=683;
    1: result:=1502;
    2: result:=3412;
    3: result:=1433;                                   {Teammode}
    4: result:=2048;                                   {Neutral - sollte eigentlich nicht auftrete }
  end;
end;

procedure IncSequNo8(var no: byte);
begin
  if no<255 then
    no:=no+1
  else
    no:=0;
end;

procedure TForm1.StopAllTimer;
begin
  timerFCHeartbeat.Enabled:=false;
  timerYGCcommandLong.Enabled:=false;
  timerTelemetry.Enabled:=false;
  timerFCCommand.Enabled:=false;
  timerGUI.Enabled:=false;
  timerSensors.Enabled:=false;
end;

procedure TForm1.ResetSensorsStatus;
begin
  gbPosition.Color:=clSensorMiss;
  gbBaro.Color:=clSensorMiss;
  gbAcc.Color:=clSensorMiss;
  gbGyro.Color:=clSensorMiss;
  gbMag.Color:=clSensorMiss;
  gbSysStatus.Color:=clSensorMiss;
  shapeGPSOK.Pen.Color:=clSensorMiss;
  shapeIMUOK.Pen.Color:=clSensorMiss;
  shapeSonar.Pen.Color:=clSensorMiss;
  shapeRSOK.Pen.Color:=clSensorMiss;
  shapeESCOK.Pen.Color:=clSensorMiss;
  picGPS.ImageIndex:=11;
  picGLONASS.ImageIndex:=11;
  picSBAS.ImageIndex:=11;
  lblOtherSats.Caption:='';
end;

procedure WriteCSVRawHeader(var list: TStringList);
var
  s: string;
  i: integer;

begin
  list.Clear;
  s:=rsTime;
  for i:=0 to 105 do
    s:=s+';'+Format('%.*d', [2, i]);
  list.Add(s);
end;

procedure SetStartValuesForGlobelVariables;
begin
  pan:=0;
  tilt:=0;
  roll:=0;
  voltage:=0;                                          {[mV]}
  SequNumberTransmit:=0;
  SequNumberReceived:=0;
  MessagesSent:=0;
  MessagesReceived:=0;
  SerialNumberFound:=false;
  starttime:=GetTickCount64;
  boottime:=0;
end;

procedure TForm1.CreateFCControl(var msg: TMavMessage; SequenceNumber: byte; const command: uint16=$FFFF);
begin
  CreateStandardPartMsg(msg, 26);
  msg.msgbytes[2]:=SequenceNumber;
  msg.msgbytes[5]:=2;                                  {TargetId Gimbal}
  msg.msgbytes[7]:=1;                                  {MsgID}

  SetUInt16ToMsg(msg, 22, InvertPanControlPosition(knPanControl.Position));
  SetUInt16ToMsg(msg, 24, tbTiltControl.Position);
  SetUInt16ToMsg(msg, 26, 2048);
  if command<$FFFF then
    SetUInt16ToMsg(msg, 28, command)
  else
    SetUInt16ToMsg(msg, 28, PanModeToInt);
  SetUInt16ToMsg(msg, 30, TiltModeToInt);
  SetUInt16ToMsg(msg, 32, 500);
  SetCRC_FE(msg);
end;

procedure TForm1.CreateFCTelemetry5GHz(var msg: TMavMessage; SequenceNumber: byte);
begin
  CreateStandardPartMsg(msg, 5);
  msg.msgbytes[2]:=SequenceNumber;
  msg.msgbytes[5]:=3;                                  {TargetId Cam}
  msg.msgbytes[7]:=2;                                  {MsgID}

  SetUInt16ToMsg(msg, 26, tilt);                       {Gimbal position to telemetry as test}
  SetUInt16ToMsg(msg, 28, roll);
  SetUInt16ToMsg(msg, 30, pan);

  if voltage>0 then begin                              { [mV] }
     msg.msgbytes[33]:=((voltage-5000) div 100) and $FF;
  end else
    msg.msgbytes[33]:=110;                             {Voltage, default 16V}

  msg.msgbytes[35]:=$FF;                               {motor_stat}
  msg.msgbytes[36]:=$41;                               {IMU_stat}
  msg.msgbytes[37]:=$55;                               {press_comp_stat}
  msg.msgbytes[38]:=16;                                {f_mode}
  msg.msgbytes[39]:=5;                                 {v_type}

  SetCRC_FE(msg);
end;

function ConnectUART(port, speed: string): string;
begin
  result:='';
  if UARTconnected then
    exit;
  UART:=TBlockSerial.Create;
  SensorStream:=TMemoryStream.Create;
  {$IFDEF LINUX}
    UART.LinuxLock:=false;
  {$ENDIF}
  UART.Connect(port);
  sleep(200);
  UART.Config(StrToIntDef(speed, defaultbaud), 8, 'N', SB1, false, false); {Config default 115200 baud, 8N1}
  if UART.LastError=0 then begin
    UARTConnected:=true;
    result:='Status: '+UART.LastErrorDesc;
  end else begin
    result:='Error: '+UART.LastErrorDesc;
  end;
end;

procedure DisconnectUART;
begin
  if UARTConnected then begin
    try
      UART.CloseSocket;
    finally
      UART.Free;
      SensorStream.Free;
      UARTConnected:=false;
    end;
  end;
end;

procedure TForm1.NumberMessagesInStatusBar;
begin
  StatusBar1.Panels[0].Text:='S: '+IntToStr(MessagesSent);
  StatusBar1.Panels[1].Text:='R: '+IntToStr(MessagesReceived);
end;

procedure TForm1.acConnectExecute(Sender: TObject);
var
  msg: TMAVmessage;
  csvlist: TStringList;

begin
  csvlist:=TStringList.Create;
  try
    msg:=Default(TMAVmessage);
    SetStartValuesForGlobelVariables;
    chTiltLineSeries1.Clear;
    chPanLineSeries1.Clear;
    chRollLineSeries1.Clear;
    GIMBALtext.Lines.Clear;
    ResetSensorsStatus;
    lblGimbalVersion.Caption:='';
    lblSerial.Caption:='';
    lblGimbalBootTime.Caption:='';
    StatusBar1.Panels[0].Text:='0';                    {Sent messages}
    StatusBar1.Panels[1].Text:='0';                    {Received messages}
    WriteCSVRawHeader(csvlist);
    StatusBar1.Panels[2].Text:=ConnectUART(cbPort.Text, cbSpeed.Text);
    If UARTconnected then begin
      StatusBar1.Panels[2].Text:=StatusBar1.Panels[2].Text+'  -  '+rsConnected;
      if pcMain.ActivePage=tsFC then
        ActAsFlightController(msg, csvlist)
      else
        if pcMain.ActivePage=tsYGC then
          ActAsGimbalChecker(msg, csvlist)
        else begin                                     {GUI as default}
          btnGeoFence.Enabled:=UARTConnected;
          btnHeightLimit.Enabled:=UARTConnected;
          ActAsGUI(msg, csvlist);
        end;

      NumberMessagesInStatusBar;
      SaveDialog1.FilterIndex:=1;
      SaveDialog1.FileName:='BCmessages_'+FormatDateTime('yyyymmdd_hhnnss', now)+'.csv';
      if cbRecord.Checked and (csvlist.Count>1) and SaveDialog1.Execute then begin
        csvlist.SaveToFile(SaveDialog1.FileName);
        StatusBar1.Panels[2].Text:=SaveDialog1.FileName+rsSaved;
      end;
    end;
  finally
    csvlist.Free;
  end;
end;

procedure TForm1.acCopySerialExecute(Sender: TObject);
begin
  if vleSystem.Cells[1, 2]<>'' then
    Clipboard.AsText:=vleSystem.Cells[1, 2];
end;

procedure TForm1.acCloseExecute(Sender: TObject);
begin
  Close;
end;

procedure TForm1.ReadGimbalPosition(msg: TMAVmessage);
begin
  if msg.msgid=3 then begin
    Read3UInt16(msg, 8, pan, tilt, roll);
  end;
end;

function TForm1.YGC_TimestampIn_ms(msg: TMAVmessage): integer;
begin
  result:=MavGetUInt32(msg, 9);
  lblGimbalBootTime.Caption:=FormatFloat(floatformat3, result/1000);
end;

procedure TForm1.GIMBAL_GYRO_POWER(msg: TMAVmessage);
begin
  YGC_TimestampIn_ms(msg);
  gridVarious.Cells[1, 1]:=FormatFloat(floatformat2, MavGetInt16(msg, 13)/100);       {GyroHope}
  gridVarious.Cells[1, 2]:=FormatFloat(floatformat2, MavGetInt16(msg, 15)/100);
  gridVarious.Cells[1, 3]:=FormatFloat(floatformat2, MavGetInt16(msg, 17)/100);

  gridVarious.Cells[1, 4]:=FormatFloat(floatformat2, MavGetInt16(msg, 19)/100);       {Gyro}
  gridVarious.Cells[1, 5]:=FormatFloat(floatformat2, MavGetInt16(msg, 21)/100);
  gridVarious.Cells[1, 6]:=FormatFloat(floatformat2, MavGetInt16(msg, 23)/100);

  gridVarious.Cells[1, 7]:=FormatFloat(floatformat2, MavGetInt16(msg, 25)/100);       {SpeedHope}
  gridVarious.Cells[1, 8]:=FormatFloat(floatformat2, MavGetInt16(msg, 27)/100);
  gridVarious.Cells[1, 9]:=FormatFloat(floatformat2, MavGetInt16(msg, 29)/100);

  gridVarious.Cells[1, 10]:=FormatFloat(floatformat2, MavGetInt16(msg, 31)/100);      {Speed}
  gridVarious.Cells[1, 11]:=FormatFloat(floatformat2, MavGetInt16(msg, 33)/100);
  gridVarious.Cells[1, 12]:=FormatFloat(floatformat2, MavGetInt16(msg, 35)/100);

  gridVarious.Cells[1, 13]:=FormatFloat(floatformat2, MavGetInt16(msg, 37)/100);      {Power}
  gridVarious.Cells[1, 14]:=FormatFloat(floatformat2, MavGetInt16(msg, 39)/100);
  gridVarious.Cells[1, 15]:=FormatFloat(floatformat2, MavGetInt16(msg, 41)/100);
end;

procedure TForm1.GIMBAL_EULER_ANGLE(msg: TMAVmessage);
var
  AngleHopeX, AngleHopeY, AngleHopeZ: int16;
  AngleX, AngleY, AngleZ: int16;

begin
  YGC_TimestampIn_ms(msg);
  AngleHopeX:=MavGetInt16(msg, 13);

  AngleHopeY:=MavGetInt16(msg, 15);
  AngleHopeZ:=MavGetInt16(msg, 17);
  gridVarious.Cells[1, 1]:=FormatFloat(floatformat2, AngleHopeX/100);
  gridVarious.Cells[1, 2]:=FormatFloat(floatformat2, AngleHopeY/100);
  gridVarious.Cells[1, 3]:=FormatFloat(floatformat2, AngleHopeZ/100);
  AngleX:=MavGetInt16(msg, 19);
  AngleY:=MavGetInt16(msg, 21);
  AngleZ:=MavGetInt16(msg, 23);
  gridVarious.Cells[1, 4]:=FormatFloat(floatformat2, AngleX/100);
  gridVarious.Cells[1, 5]:=FormatFloat(floatformat2, AngleY/100);
  gridVarious.Cells[1, 6]:=FormatFloat(floatformat2, AngleZ/100);

  gridVarious.Cells[1, 7]:=FormatFloat(floatformat2, MavGetInt16(msg, 25)/100);  {EulerHope}
  gridVarious.Cells[1, 8]:=FormatFloat(floatformat2, MavGetInt16(msg, 27)/100);
  gridVarious.Cells[1, 9]:=FormatFloat(floatformat2, MavGetInt16(msg, 29)/100);

  gridVarious.Cells[1, 10]:=FormatFloat(floatformat2, MavGetInt16(msg, 31)/100); {Euler}
  gridVarious.Cells[1, 11]:=FormatFloat(floatformat2, MavGetInt16(msg, 33)/100);
  gridVarious.Cells[1, 12]:=FormatFloat(floatformat2, MavGetInt16(msg, 35)/100);

  gridVarious.Cells[1, 13]:=FormatFloat(floatformat2, (AngleHopeX-AngleX)/100);  {AngleError}
  gridVarious.Cells[1, 14]:=FormatFloat(floatformat2, (AngleHopeY-AngleY)/100);
  gridVarious.Cells[1, 15]:=FormatFloat(floatformat2, (AngleHopeZ-AngleZ)/100);
end;

procedure TForm1.GIMBAL_ACC(msg: TMAVmessage);
begin
  YGC_TimestampIn_ms(msg);
  gridVarious.Cells[1, 1]:=FormatFloat(floatformat3, MavGetInt16(msg, 13)/1000); {AccLowPass}
  gridVarious.Cells[1, 2]:=FormatFloat(floatformat3, MavGetInt16(msg, 15)/1000);
  gridVarious.Cells[1, 3]:=FormatFloat(floatformat3, MavGetInt16(msg, 17)/1000);
  gridVarious.Cells[1, 4]:=FormatFloat(floatformat3, MavGetInt16(msg, 19)/1000);

  gridVarious.Cells[1, 5]:=FormatFloat(floatformat3, MavGetInt16(msg, 21)/1000); {Acc}
  gridVarious.Cells[1, 6]:=FormatFloat(floatformat3, MavGetInt16(msg, 23)/1000);
  gridVarious.Cells[1, 7]:=FormatFloat(floatformat3, MavGetInt16(msg, 25)/1000);
  gridVarious.Cells[1, 8]:=FormatFloat(floatformat3, MavGetInt16(msg, 27)/1000);

  gridVarious.Cells[1, 9]:=FormatFloat(floatformat3, MavGetInt16(msg, 29)/1000); {HorizontalAcc}
  gridVarious.Cells[1, 10]:=FormatFloat(floatformat3, MavGetInt16(msg, 31)/1000); {VerticalAcc}

  gridVarious.Cells[1, 11]:=FormatFloat(floatformat3, MavGetInt16(msg, 33)/1000); {AccKp}
  gridVarious.Cells[1, 12]:=FormatFloat(floatformat3, MavGetInt16(msg, 35)/1000);
  gridVarious.Cells[1, 13]:=FormatFloat(floatformat3, MavGetInt16(msg, 37)/1000);

  gridVarious.Cells[1, 14]:=FormatFloat(floatformat2, MavGetInt16(msg, 39)/100); {HorizontalAccChage}
  gridVarious.Cells[1, 15]:=FormatFloat(floatformat2, MavGetInt16(msg, 41)/100);
end;

procedure TForm1.GIMBAL_TEMP_DIFF(msg: TMAVmessage);
begin
  YGC_TimestampIn_ms(msg);
  gridVarious.Cells[1, 1]:=FormatFloat(floatformat3, MavGetInt16(msg, 13)/1000); {HistoryZroTolerance}
  gridVarious.Cells[1, 2]:=FormatFloat(floatformat3, MavGetInt16(msg, 15)/1000);
  gridVarious.Cells[1, 3]:=FormatFloat(floatformat3, MavGetInt16(msg, 17)/1000);

  gridVarious.Cells[1, 4]:=FormatFloat(floatformat3, MavGetInt16(msg, 19)/1000); {ZroTolerance}
  gridVarious.Cells[1, 5]:=FormatFloat(floatformat3, MavGetInt16(msg, 21)/1000);
  gridVarious.Cells[1, 6]:=FormatFloat(floatformat3, MavGetInt16(msg, 23)/1000);

  gridVarious.Cells[1, 7]:=FormatFloat(floatformat3, MavGetInt16(msg, 25)/1000); {ZroVariation}
  gridVarious.Cells[1, 8]:=FormatFloat(floatformat3, MavGetInt16(msg, 27)/1000);
  gridVarious.Cells[1, 9]:=FormatFloat(floatformat3, MavGetInt16(msg, 29)/1000);

  gridVarious.Cells[1, 10]:=FormatFloat(floatformat2, MavGetInt16(msg, 31)/100); {Temp}
  gridVarious.Cells[1, 11]:=FormatFloat(floatformat2, MavGetInt16(msg, 33)/100);
  gridVarious.Cells[1, 12]:=FormatFloat(floatformat2, MavGetInt16(msg, 35)/100);
  gridVarious.Cells[1, 13]:=FormatFloat(floatformat2, MavGetInt16(msg, 37)/100);
end;

procedure TForm1.GIMBAL_STATUS(msg: TMAVmessage);
begin
  YGC_TimestampIn_ms(msg);
  gridStatus.Cells[1, 1]:=FormatFloat(floatformat2, MavGetUInt16(msg, 13)/100);
  gridStatus.Cells[1, 2]:=FormatFloat(floatformat2, MavGetUInt16(msg, 15)/1000);
  gridStatus.Cells[1, 3]:=IntToStr(MavGetUInt16(msg, 17));                       {Seconds}

  gridStatus.Cells[1, 4]:=IntToStr(MavGetInt16(msg, 19));                        {Enc_data}
  gridStatus.Cells[1, 5]:=IntToStr(MavGetInt16(msg, 21));
  gridStatus.Cells[1, 6]:=IntToStr(MavGetInt16(msg, 23));

  gridStatus.Cells[1, 7]:=FormatFloat(floatformat2, MavGetInt16(msg, 25)/100);   {StageAngle}
  gridStatus.Cells[1, 8]:=FormatFloat(floatformat2, MavGetInt16(msg, 27)/100);

  gridStatus.Cells[1, 9]:=FormatFloat(floatformat2, MavGetInt16(msg, 29)/100);   {AC_angle}
  gridStatus.Cells[1, 10]:=FormatFloat(floatformat2, MavGetInt16(msg, 31)/100);
  gridStatus.Cells[1, 11]:=FormatFloat(floatformat2, MavGetInt16(msg, 33)/100);

  gridStatus.Cells[1, 12]:=IntToStr(msg.msgbytes[35]);                           {GyroStable}
  gridStatus.Cells[1, 13]:=IntToStr(msg.msgbytes[36]);
  gridStatus.Cells[1, 14]:=IntToStr(msg.msgbytes[37]);
end;

procedure TForm1.Channel_data(msg: TMAVmessage);
var
  i: byte;

begin
  YGC_TimestampIn_ms(msg);
  for i:=0 to 7 do
    gridVarious.Cells[1, i+1]:=IntToStr(MavGetUInt16(msg, i*2+13));
end;

procedure TForm1.CAM_SERIAL(msg: TMAVmessage);
var
  s: string;

begin
  if SerialNumberFound then
    exit;
  s:=GetSERIAL(msg, LengthFixPartFE+1);
  lblSerial.Caption:=s;
  Caption:=Application.Title+tab2+s;
  SerialNumberFound:=true;                                           {Read serial number only once}
end;

procedure TForm1.TEXT_MESSAGE(msg: TMAVmessage);
var
  s: string;

begin
  s:=GetTEXT_MESSAGE(msg);
  GIMBALtext.Lines.Add(s);
end;

procedure TForm1.SendYGCHeartbeat;
var
  msg: TMAVmessage;

begin
  if UARTConnected then begin
    CreateYGCcommandMessage(msg);
    SendUARTMessage(msg, LengthFixPartFE);
  end;
  NumberMessagesInStatusBar;
end;

procedure TForm1.ReadYGCcameraMessages(msg: TMAVmessage);
begin
  if msg.sysid=2 then begin
    if msg.msgid=0 then
      SendYGCHeartbeat
    else
    if (msg.msgid=2) and (msg.targetid=YGCsysID) then begin
      case msg.msgbytes[8] of             {YGC message type}
        1: if rgYGC_Type.ItemIndex=0 then GIMBAL_GYRO_POWER(msg);
        2: if rgYGC_Type.ItemIndex=1 then GIMBAL_EULER_ANGLE(msg);
        3: if rgYGC_Type.ItemIndex=2 then GIMBAL_ACC(msg);
        4: if rgYGC_Type.ItemIndex=4 then Channel_data(msg);
        5: if rgYGC_Type.ItemIndex=3 then GIMBAL_TEMP_DIFF(msg);
        6: GIMBAL_STATUS(msg);
        $12: CAM_SERIAL(msg);
        $FE: TEXT_MESSAGE(msg);
      else
        GIMBALtext.Lines.Add('Unknown YGC message type: 0x'+
                        IntToHex(msg.msgbytes[8], 2)+tab2+
                        ' = '+IntToStr(msg.msgbytes[8]));
      end;
    end;
  end;
end;

procedure TForm1.RecordMessage(msg: TMAVmessage; list: TStringList; LengthFixPart: byte);
var
  s: string;
  i: integer;

begin
  s:=FormatFloat(floatformat3, (GetTickCount64-starttime)/1000);
  for i:=0 to msg.msglength+LengthFixPart+1 do begin
    s:=s+';'+IntToHex(msg.msgbytes[i], 2);
  end;
  list.Add(s);
end;

function Uint16ToInt16(v: UInt16): int16;
begin
  result:=v and $7F;
  if (v and $80)>0 then
    result:=-result;
end;

procedure TForm1.FillCharts;
begin
  if tilt<>0 then begin
    chPanLineSeries1.AddXY(MessagesReceived, GimbalPanToDegree(pan));
    chTiltLineSeries1.AddXY(MessagesReceived, GimbalAngleToDegree(tilt));
    chRollLineSeries1.AddXY(MessagesReceived, GimbalAngleToDegree(roll));
  end;
end;

procedure SendGUIParamRequest;
var
  msg: TMAVmessage;

begin
  CreateGUI_PARAM_REQUEST_LIST(msg);
//  SendUARTMessage(msg, LengthFixPartBC);           {Not sending reduces text messages}
  CreateGUI_MISSION_REQUEST_INT(msg, 1);
  SendUARTMessage(msg, LengthFixPartBC);
  CreateGUI_MISSION_REQUEST_INT(msg, 11);
  SendUARTMessage(msg, LengthFixPartBC);
  CreateGUI_MISSION_REQUEST_INT(msg, 12);
  SendUARTMessage(msg, LengthFixPartBC);
end;

procedure TForm1.FillGUIPosition24(const sats: TGPSdata);
begin
  vlePosition.Cells[1, 1]:=FormatCoordinates(sats.lat);
  vlePosition.Cells[1, 2]:=FormatCoordinates(sats.lon);
  vlePosition.Cells[1, 3]:=FormatAltitude(sats.altMSL);

  if rgAddValue.ItemIndex=6 then
    AddToChart(boottime, sats.altMSL/1000);

  vlePosition.Cells[1, 6]:=IntToStr(sats.sats_inuse);
  vlePosition.Cells[1, 7]:=FormatDOP(sats.eph);
  vlePosition.Cells[1, 8]:=FormatDOP(sats.epv);
  vlePosition.Cells[1, 9]:=FixTypeToStr(sats.fix_type);

  vleVelocity.Cells[1, 1]:=FormatSpeed(sats.vel);
end;

{Sat ID are PRN numbers.
 See https://continuouswave.com/forum/viewtopic.php?t=1696
 SBAS: https://gssc.esa.int/navipedia/index.php/SBAS_Fundamentals}

procedure TForm1.FillGPS_STATUS(const sats: TGPSdata);
begin
  if sats.numGPS_visible>0 then
    picGPS.ImageIndex:=10
  else
    picGPS.ImageIndex:=11;
  if sats.numGLONASS_visible>0 then
    picGLONASS.ImageIndex:=10
  else
    picGLONASS.ImageIndex:=11;
  if sats.numSBAS_visible>0 then
    picSBAS.ImageIndex:=10
  else
    picSBAS.ImageIndex:=11;
  if sats.numOther_visible>0 then
    lblOtherSats.Caption:='Other Sat-PRNs: '+IntToStr(sats.numOther_visible)
  else
    lblOtherSats.Caption:='';

  vlePosition.Cells[1, 5]:=IntToStr(sats.sats_visible);
  if (gbPosition.Color<>clSensorOK) and (gbPosition.Color<>clSatUsed) then
    gbPosition.Color:=clSensorOK;
  if shapeGPSOK.Pen.Color<>clSensorOK then
    shapeGPSOK.Pen.Color:=clSensorOK;
end;

procedure TForm1.FillGUIPosition33(const sats: TGPSdata);
begin
  vlePosition.Cells[1, 1]:=FormatCoordinates(sats.lat);
  vlePosition.Cells[1, 2]:=FormatCoordinates(sats.lon);
  vlePosition.Cells[1, 4]:=FormatAltitude(sats.alt_rel);
  vleBaro.Cells[1, 3]:=FormatAltitude(sats.alt_rel);

  if rgAddValue.ItemIndex=7 then
    AddToChart(boottime, sats.alt_rel/1000);

  vleVelocity.Cells[1, 2]:=FormatXYZSpeed(sats.vx);
  vleVelocity.Cells[1, 3]:=FormatXYZSpeed(sats.vy);
  vleVelocity.Cells[1, 4]:=FormatXYZSpeed(sats.vz);
end;

procedure TForm1.FillGUIIMU(const data: THWstatusData);
var
  Magnitude: double;

begin
  vleAcc.Cells[1, 1]:=IntToStr(data.AccX);
  vleAcc.Cells[1, 2]:=IntToStr(data.AccY);
  vleAcc.Cells[1, 3]:=IntToStr(data.AccZ);
  Magnitude:=Value3D(data.AccX, data.AccY, data.AccZ);
  vleAcc.Cells[1, 4]:=FormatFloat(floatformat2, Magnitude);
  if rgAddValue.ItemIndex=5 then
    AddToChart(boottime, Magnitude);

  vleGyro.Cells[1, 1]:=IntToStr(data.GyroX);
  vleGyro.Cells[1, 2]:=IntToStr(data.GyroY);
  vleGyro.Cells[1, 3]:=IntToStr(data.GyroZ);

  vleMag.Cells[1, 1]:=IntToStr(data.MagX);
  vleMag.Cells[1, 2]:=IntToStr(data.MagY);
  vleMag.Cells[1, 3]:=IntToStr(data.MagZ);
end;

procedure TForm1.vleAccPrepareCanvas(Sender: TObject; aCol, aRow: Integer;
  aState: TGridDrawState);
begin
  if aCol=1 then begin
    if (aRow=4) and (vleAcc.Cells[1, 4]<>'') and
       (StrToFloat(vleAcc.Cells[1, 4])<AccMin)  then
      CellColorSetting(vleAcc,clSensorMiss);

  end;
end;

procedure TForm1.FillSENSOR_OFFSETS(const data: THWstatusData);
begin
  vleAcc.Cells[1, 5]:=FormatFloat(floatformat3, data.AccCaliX);
  vleAcc.Cells[1, 6]:=FormatFloat(floatformat3, data.AccCaliY);
  vleAcc.Cells[1, 7]:=FormatFloat(floatformat3, data.AccCaliZ);

  vleGyro.Cells[1, 4]:=FormatFloat(floatformat3, data.GyroCaliX);
  vleGyro.Cells[1, 5]:=FormatFloat(floatformat3, data.GyroCaliY);
  vleGyro.Cells[1, 6]:=FormatFloat(floatformat3, data.GyroCaliZ);

  vleMag.Cells[1, 5]:=IntToStr(data.MagOfsX);
  vleMag.Cells[1, 6]:=IntToStr(data.MagOfsY);
  vleMag.Cells[1, 7]:=IntToStr(data.MagOfsZ);
end;

procedure TForm1.FillAttitude(data: TAttitudeData);
begin
  vleOrientation.Cells[1, 1]:=FormatFloat(floatformat1, data.roll);
  vleOrientation.Cells[1, 2]:=FormatFloat(floatformat1, data.pitch);
  vleOrientation.Cells[1, 3]:=FormatFloat(floatformat1, data.yaw);
  case rgAddValue.ItemIndex of
    2: AddToChart(boottime, data.roll);
    3: AddToChart(boottime, data.pitch);
    4: AddToChart(boottime, data.yaw);
  end;
end;

procedure TForm1.FillEKF_STATUS_REPORT(data: TAttitudeData);
begin
  vleVelocity.Cells[1, 5]:=FormatFloat(floatformat3, data.velocity_variance);
  vleMag.Cells[1, 4]:=FormatFloat(floatformat3, data.compass_variance);
  vleSysStatus.Cells[1, 7]:=IntToHex(data.EKFstatus, 4);
end;

{ YTH OK w/o RS: 00 A0 FC 2F      with RS: 02 A0 FC 6F

Flags from low to high:

1    MAV_SYS_STATUS_SENSOR_3D_GYRO=1                     0x01 3D gyro
1    MAV_SYS_STATUS_SENSOR_3D_ACCEL=2                    0x02 3D accelerometer
1    MAV_SYS_STATUS_SENSOR_3D_MAG=4                      0x04 3D magnetometer
1    MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE=8           0x08 absolute pressure

0    MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE=16      0x10 differential pressure
1    MAV_SYS_STATUS_SENSOR_GPS=32                        0x20 GPS
0(1) MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW=64               0x40 optical flow
0    MAV_SYS_STATUS_SENSOR_VISION_POSITION=128           0x80 computer vision position

0    MAV_SYS_STATUS_SENSOR_LASER_POSITION=256            0x100 laser based position
0    MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH=512     0x200 external ground truth (Vicon or Leica)
1    MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL=1024     0x400 3D angular rate control
1    MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION=2048   0x800 attitude stabilization

1    MAV_SYS_STATUS_SENSOR_YAW_POSITION=4096             0x1000 yaw position
1    MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL=8192       0x2000 z/altitude control
1    MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL=16384     0x4000 x/y position control
1    MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS=32768           0x8000 motor outputs / control

0(1) MAV_SYS_STATUS_SENSOR_RC_RECEIVER=65536             0x10000 Radio SR24
0    MAV_SYS_STATUS_SENSOR_3D_GYRO2=131072               0x20000 2nd 3D gyro
0    MAV_SYS_STATUS_SENSOR_3D_ACCEL2=262144              0x40000 2nd 3D accelerometer
0    MAV_SYS_STATUS_SENSOR_3D_MAG2=524288                0x80000 2nd 3D magnetometer

0    MAV_SYS_STATUS_GEOFENCE=1048576                     0x100000 geofence
1    MAV_SYS_STATUS_AHRS=2097152                         0x200000 AHRS subsystem health (Attitude Heading Reference System)
0    MAV_SYS_STATUS_TERRAIN=4194304                      0x400000 Terrain subsystem health
1    MAV_SYS_STATUS_REVERSE_MOTOR=8388608                0x800000 Motors are reversed   Sonar !

0    MAV_SYS_STATUS_LOGGING=16777216                     0x1000000 Logging
0(1) MAV_SYS_STATUS_SENSOR_BATTERY=33554432              0x2000000 Battery
0    MAV_SYS_STATUS_SENSOR_PROXIMITY=67108864            0x4000000 Proximity
0    MAV_SYS_STATUS_SENSOR_SATCOM=134217728              0x8000000 Satellite Communication

0    MAV_SYS_STATUS_PREARM_CHECK=268435456               0x10000000 pre-arm check status. Always healthy when armed
0    MAV_SYS_STATUS_OBSTACLE_AVOIDANCE=536870912         0x20000000 Avoidance/collision prevention
0    MAV_SYS_STATUS_SENSOR_PROPULSION=1073741824         0x40000000 propulsion (actuator, esc, motor or propellor)
0    MAV_SYS_STATUS_EXTENSION_USED=2147483648            0x80000000 Extended bit-field are used for further sensor status bits (needs to be set in onboard_control_sensors_present only)
}

procedure TForm1.FillGUI_SYS_STATUS(const msg: TMAVmessage; var data: TGPSdata);
var
  err: integer;
  healty, enabld: UInt32;

begin
  vleSysStatus.Cells[1, 1]:=IntToHexSpace(MavGetUInt32(msg, LengthFixPartBC));
  enabld:=MavGetUInt32(msg, LengthFixPartBC+4);
  vleSysStatus.Cells[1, 2]:=IntToHexSpace(enabld);
  healty:=MavGetUInt32(msg, LengthFixPartBC+8);
  vleSysStatus.Cells[1, 3]:=IntToHexSpace(healty);

  vleSysStatus.Cells[1, 4]:=IntToStr(MavGetUInt16(msg, LengthFixPartBC+18));
  vleSysStatus.Cells[1, 5]:=IntToStr(MavGetUInt16(msg, LengthFixPartBC+20));
  err:=MavGetUInt16(msg, LengthFixPartBC+22)+MavGetUInt16(msg, LengthFixPartBC+24)+
       MavGetUInt16(msg, LengthFixPartBC+26)+MavGetUInt16(msg, LengthFixPartBC+28);
  vleSysStatus.Cells[1, 6]:=IntToStr(err);
  vleSysStatus.Cells[1, 8]:=FormatFloat(floatformat2, data.voltage/1000)+'V';
  if data.batt_cap<max8 then
    vleSysStatus.Cells[1, 8]:=vleSysStatus.Cells[1, 8]+'  ('+IntToStr(data.batt_cap)+'%)';

  if (enabld and $10000)=$10000 then                   {Radio SR24}
    vleSysStatus.Cells[1, 9]:=rsEnabled;
  if (healty and $10000)=$10000 then
    vleSysStatus.Cells[1, 9]:=rsConnected;

  if (enabld and $40)=$40 then
    vleSystem.Cells[1, 5]:=rsAvailable
  else
    vleSystem.Cells[1, 5]:=rsNotMounted;
  if (healty and $2000040)=$2000040 then begin         {Real Sense}
    if shapeRSOK.Pen.Color<>clSensorOK then
      shapeRSOK.Pen.Color:=clSensorOK;
  end;

  if (healty and 1)=1 then begin
    if gbGyro.Color<>clSensorOK then
      gbGyro.Color:=clSensorOK;
  end else begin
    data.sensors_OK:=false;
    gbGyro.Color:=clSensorMiss;
  end;
  if (healty and 2)=2 then begin
    if gbAcc.Color<>clSensorOK then
      gbAcc.Color:=clSensorOK;
  end else begin
    data.sensors_OK:=false;
    gbAcc.Color:=clSensorMiss;
  end;
  if (healty and 4)=4 then begin
    if gbMag.Color<>clSensorOK then
      gbMag.Color:=clSensorOK;
  end else begin
    data.sensors_OK:=false;
    gbMag.Color:=clSensorMiss;
  end;
  if (healty and $8000)=$8000 then begin
    if shapeESCOK.Pen.Color<>clSensorOK then
      shapeESCOK.Pen.Color:=clSensorOK;
  end else begin
    shapeESCOK.Pen.Color:=clSensorMiss;
  end;
  if (healty and $0F)=$0F then begin                   {Gyro, Acc, Mag and Baro}
    if shapeIMUOK.Pen.Color<>clSensorOK then
      shapeIMUOK.Pen.Color:=clSensorOK;
  end else begin
    data.sensors_OK:=false;
    shapeIMUOK.Pen.Color:=clSensorMiss;
  end;

  if (healty and 8)=8 then begin
    if gbBaro.Color<>clSensorOK then
      gbBaro.Color:=clSensorOK;
  end else begin
    data.sensors_OK:=false;
    gbBaro.Color:=clSensorMiss;
  end;
  if (healty and $800000)=$800000 then begin
    if shapeSonar.Pen.Color<>clSensorOK then
      shapeSonar.Pen.Color:=clSensorOK;
  end else begin
    data.sensors_OK:=false;
    shapeSonar.Pen.Color:=clSensorMiss;
  end;

  if data.sensors_OK and (err=0) then
    gbSysStatus.Color:=clSensorOK
  else
    gbSysStatus.Color:=clSensorMiss;
end;

procedure TForm1.ClearGUI;
var
  i: byte;

begin
  SerialNumberFound:=false;
  btnTurnAll.Enabled:=false;
  BarSatSNR.Clear;
  SatPolarSeries.Clear;
  ResetSensorsStatus;
  AddValueLineSeries1.Clear;
  lblSysTime.Caption:=rsTimeUTC;
  GUItext.Text:='';
  acCopySerial.Enabled:=false;
  for i:=1 to vleSystem.RowCount-1 do
    vleSystem.Cells[1, i]:='';
end;

function FormatBootTime(const data: TGPSdata): string;
begin
  result:=FormatDateTime(timezzz, data.boottime);
end;

procedure TForm1.AddToChart(const Time: TDateTime; value: single);
begin
  chAddValue.BeginUpdateBounds;
  if time>0 then begin
    if AddValueLineSeries1.Count>1000 then
      AddValueLineSeries1.Delete(0);
    AddValueLineSeries1.AddXY(time, value);
  end;
  chAddValue.EndUpdateBounds;
end;

procedure TForm1.ReadGUIMessages(msg: TMAVmessage);
var
  GUI_GPSdata: TGPSdata;
  Sensors: THWstatusData;
  DronePos: TAttitudeData;
  Values24: TData96;

  s, dt: string;
  value: single;
(*
  procedure DATA96;     {test only}
  var
    w: single;
    i: byte;
    s: string;

  begin
    s:=lblFCtime.Caption;
    for i:=0 to 23 do begin
      w:=MavGetFloat(msg, i*4+8);
      s:=s+';'+FormatFloat('0,000', w);
    end;
    GUItext.Lines.Add(s);
  end;
*)

begin
  GUI_GPSdata:=Default(TGPSdata);
  Sensors:=Default(THWstatusData);
  DronePos:=Default(TAttitudeData);
  Values24:=Default(TData96);
  BeginFormUpdate;

  case msg.msgid of
    0: begin
      SendGUIParamRequest;
      lblOK.Caption:=tab1;
    end;
    1: begin
      SYS_STATUS(msg, LengthFixPartBC, GUI_GPSdata);
      FillGUI_SYS_STATUS(msg, GUI_GPSdata);
    end;
    2: begin
      SYS_TIME(msg, LengthFixPartBC, GUI_GPSdata);
      lblSysTime.Caption:=FormatDateTime(timefull, GUI_GPSdata.timeUTC);
    end;

    22: begin
      s:=PARAM_VALUE(msg, LengthFixPartBC, value);
//      GUItext.Lines.Add(s+' = '+FormatFloat('0', value)); {Option: Let's look what else may come}
      if s=pGeoFence then
        lblGeoFenceVal.Caption:=FormatFloat('0', value)+'m'
      else
        if s=pHeightLimit then
          lblHeightLimitVal.Caption:=FormatFloat('0', value)+'m';
    end;

    24: begin
      GPS_RAW_INT(msg, LengthFixPartBC, GUI_GPSdata);
      FillGUIposition24(GUI_GPSdata);
      timerSensors.Enabled:=false;
    end;
    25: begin
      GPS_STATUS(msg, LengthFixPartBC, GUI_GPSdata);
      FillGPS_STATUS(GUI_GPSdata);
      CreateSatSNRBarChart(GUI_GPSdata);
      CreateSatPolarDiagram(GUI_GPSdata);
    end;

    27: begin
      RAW_IMU(msg, LengthFixPartBC, sensors);
      FillGUIIMU(sensors);
    end;

    29: begin
      SCALED_PRESSURE(msg, LengthFixPartBC, Sensors);
      vleBaro.Cells[1, 1]:=FormatFloat(floatformat2, Sensors.pressure_abs)+'hPa';
      vleBaro.Cells[1, 2]:=FormatFloat(floatformat2, Sensors.baro_temp/100)+'°C';
      if rgAddvalue.ItemIndex=1 then
        AddToChart(boottime, Sensors.baro_temp/100);
    end;
    30: begin
      ATTITUDE(msg, LengthFixPartBC, DronePos);
      FillAttitude(DronePos);
    end;

    33: begin
      GLOBAL_POSITION_INT(msg, LengthFixPartBC, GUI_GPSdata);
      boottime:=GUI_GPSdata.boottime;
      lblFCtime.Caption:=FormatBootTime(GUI_GPSdata);
      lblFCtimeGPS.Caption:=FormatBootTime(GUI_GPSdata);
      FillGUIposition33(GUI_GPSdata);
    end;

    52: begin
      vleSystem.Cells[1, 1]:=GetSYSTEM(msg, LengthFixPartBC, s, dt);
      vleSystem.Cells[1, 3]:=s;
      vleSystem.Cells[1, 4]:=dt;

    end;

    56: begin
      if not SerialNumberFound then begin
        s:=GetSERIAL(msg, LengthFixPartBC);
        vleSystem.Cells[1, 2]:=s;
        Caption:=Application.Title+tab2+s;
        SerialNumberFound:=true;
        acCopySerial.Enabled:=true;
        btnDisableNFZ.Enabled:=UARTConnected;
        btnEnableNFZ.Enabled:=UARTConnected;
      end;
    end;
    58: if msg.msgbytes[7]=1 then
      lblOK.Caption:='OK';

    150: begin
      if vleGyro.Cells[1, 4]='' then begin
        SENSOR_OFFSETS(msg, LengthFixPartBC, Sensors);
        FillSENSOR_OFFSETS(sensors);
      end;
    end;

    172: begin
      DATA96(msg, LengthFixPartBC, Values24);
      vleGyro.Cells[1, 7]:=FormatFloat(floatformat2, Values24.value[0])+'°C';
      if rgAddValue.ItemIndex=0 then
        AddToChart(boottime, Values24.value[0]);
    end;

    193: begin
      EKF_STATUS_REPORT(msg, LengthFixPartBC, DronePos);
      FillEKF_STATUS_REPORT(DronePos);
    end;

    253: GUItext.Lines.Add(STATUSTEXT(msg, LengthFixPartBC, ' '));
  end;
  EndFormUpdate;
end;

procedure TForm1.ActAsGUI(var msg: TMAVmessage; list: TStringList);
begin
  ClearGUI;
  SerialNumberFound:=false;
  timerGUI.Enabled:=true;
  acEnableTesting.Enabled:=true;
  while (UART.LastError=0) and UARTConnected do begin
    if UART.CanRead(0) then begin
      ReadMessage_BC(msg);
      if msg.valid then begin
        ReadGUIMessages(msg);
        if cbRecord.Checked then
          RecordMessage(msg, list, LengthFixPartBC);
        inc(MessagesReceived);
      end;
    end;
    if cbLimitMsg.Checked and (GUItext.Lines.Count>600) then
      GUItext.Lines.Clear;
    Application.ProcessMessages;
  end;
end;

procedure TForm1.ActAsGimbalChecker(var msg: TMAVmessage; list: TStringList);
begin
  ClearMessageTables;
  timerYGCcommandLong.Enabled:=true;
  while (UART.LastError=0) and UARTConnected do begin
    if UART.CanRead(0) then begin
      ReadMessage_FE(msg);
      if msg.valid then begin
        ReadYGCcameraMessages(msg);

//        ReadGimbalPosition(msg);
//        FillCharts;
        if cbRecord.Checked then
          RecordMessage(msg, list, LengthFixPartFE);
        if cbSensor.Checked then
          SensorStream.WriteBuffer(msg.msgbytes, msg.msglength+LengthFixPartBC+2);
        inc(MessagesReceived);
      end;
    end;
    Application.ProcessMessages;
  end;
end;

procedure TForm1.ActAsFlightController(var msg: TMAVmessage; list: TStringList);
begin
  timerFCHeartbeat.Enabled:=true;
  sleep(100);
  timerFCcommand.Enabled:=true;
  if cbTelemetry.Checked then begin
    sleep(100);
    timerTelemetry.Enabled:=true;                      {Test telemetry only}
  end;
  while (UART.LastError=0) and UARTConnected do begin
    if UART.CanRead(0) then begin
      ReadMessage_FE(msg);
      if msg.valid then begin
        ReadGimbalPosition(msg);
        if lblGimbalVersion.Caption='' then
          lblGimbalVersion.Caption:=GetGIMBAL_FW_VERSION(msg);
        FillCharts;
        if cbRecord.Checked then
          RecordMessage(msg, list, LengthFixPartFE);
        inc(MessagesReceived);
      end;
    end;
    Application.ProcessMessages;
  end;
end;

procedure TForm1.ReadMessage_BC(var msg: TMAVmessage);
var
  b, len: byte;
  i: integer;

begin
  msg.valid:=false;
  repeat
    b:=UART.RecvByte(timeout);
  until (b=MagicBC) or (UART.LastError<>0) or (not UARTConnected);
  msg.msgbytes[0]:=b;
  len:=UART.RecvByte(timeout);
  msg.msgbytes[1]:=len;                                {Message length}
  msg.msglength:=len;
  b:=UART.RecvByte(timeout);
  msg.msgbytes[2]:=b;                                  {Sequ number}
  b:=UART.RecvByte(timeout);
  if b<>1 then
    exit;
  msg.msgbytes[3]:=b;                                  {SysID}
  b:=UART.RecvByte(timeout);
  if b<>1 then
    exit;
  msg.msgbytes[4]:=b;                                  {TargetID}
   b:=UART.RecvByte(timeout);
  msg.msgbytes[5]:=b;                                  {MsgID}
  msg.msgid:=b;
  for i:=6 to len+LengthFixPartBC+1 do begin
    msg.msgbytes[i]:=UART.RecvByte(timeout);
  end;
  if CheckCRC16X25(msg, LengthFixPartBC) then begin
    msg.sysid:=msg.msgbytes[3];
    msg.targetid:=msg.msgbytes[4];
    msg.msgid:=msg.msgbytes[5];
    msg.valid:=true;
  end;
end;

procedure TForm1.ReadMessage_FE(var msg: TMAVmessage);
var
  b, len: byte;
  i: integer;

begin
  msg.valid:=false;
  repeat
    b:=UART.RecvByte(timeout);
  until (b=MagicFE) or (UART.LastError<>0) or (not UARTConnected);
  msg.msgbytes[0]:=b;
  len:=UART.RecvByte(timeout);
  msg.msgbytes[1]:=len;                               {Message length}
  msg.msglength:=len;
  b:=UART.RecvByte(timeout);
  msg.msgbytes[2]:=b;                                 {Sequ number}
  b:=UART.RecvByte(timeout);
  if b>10 then
    exit;
  msg.msgbytes[3]:=b;                                 {SysID}
  b:=UART.RecvByte(timeout);
  if b>0 then
    exit;
  msg.msgbytes[4]:=b;                                 {CompID}
  b:=UART.RecvByte(timeout);
  msg.msgbytes[5]:=b;                                 {TargetID}
  b:=UART.RecvByte(timeout);
  if b>0 then
    exit;
  msg.msgbytes[6]:=b;                                 {SubTargetID}
  for i:=7 to len+LengthFixPartFE+1 do begin
    msg.msgbytes[i]:=UART.RecvByte(timeout);
  end;
  if CheckCRC16MAV(msg, LengthFixPartFE) then begin
    msg.sysid:=msg.msgbytes[3];
    msg.targetid:=msg.msgbytes[5];
    msg.msgid:=msg.msgbytes[7];
    msg.valid:=true;
  end;
end;

procedure TForm1.acDisconnectExecute(Sender: TObject);
begin
  StopAllTimer;
  DisconnectUART;
  btnGeoFence.Enabled:=UARTConnected;
  btnHeightLimit.Enabled:=UARTConnected;
  btnDisableNFZ.Enabled:=UARTConnected;
  btnEnableNFZ.Enabled:=UARTConnected;
  acEnableTesting.Enabled:=UARTConnected;
  StatusBar1.Panels[2].Text:=rsDisconnected;
end;

procedure TForm1.acEnableTestingExecute(Sender: TObject);
begin
  btnTurnAll.Enabled:=false;
  if UARTconnected then begin
    if MessageDlg(rsConfirm, msgPropellerRemoved,
                  mtConfirmation, [mbYes, mbNo], 0, mbNo)=mrYes then begin
      btnTurnAll.Enabled:=true;
    end;
  end;
end;

procedure TForm1.acSaveGUItextExecute(Sender: TObject);
begin
  SaveDialog1.Title:=hntSaveGUItext;
  SaveDialog1.FilterIndex:=2;
  SaveDialog1.FileName:='TextMessages_'+FormatDateTime('yyyymmdd_hhnnss', now)+'.txt';
  if SaveDialog1.Execute then begin
    if GUItext.Lines.Count>0 then begin
      GUItext.Lines.SaveToFile(SaveDialog1.FileName);
      StatusBar1.Panels[2].Text:=SaveDialog1.FileName+rsSaved;
      GUItext.Lines.Clear;
    end;
  end;
end;

procedure TForm1.acScanPortsExecute(Sender: TObject);
var
{$IFDEF UNIX}
  cmd: TProcess;
  list: TStringList;
{$ENDIF}
  i: integer;

begin
{$IFDEF WINDOWS}
  cbPort.Text:='';
  cbPort.Items.Clear;
  cbPort.Items.CommaText:=GetSerialPortNames;
  if cbPort.Items.Count>0 then begin
    cbPort.Text:=cbPort.Items[cbPort.Items.Count-1];
    for i:=0 to  cbPort.Items.Count-1 do begin
      GUItext.Lines.Add(cbPort.Items[i]);              {Make for Win same as for LINUX}
      GIMBALtext.Lines.Add(cbPort.Items[i]);
    end;
    StatusBar1.Panels[2].Text:=cbPort.Items[i];
  end else
    StatusBar1.Panels[2].Text:=errNoUSBport;

{$ENDIF}
{$IFDEF UNIX}
  cmd:=TProcess.Create(nil);
  list:=TStringList.Create;
  try
    GIMBALtext.Lines.Clear;
    cmd.Options:=cmd.Options+[poWaitOnExit, poUsePipes];
    cmd.Executable:='ls';
    for i:=0 to cbPort.Items.count-1 do begin
      cmd.Parameters.Clear;
      cmd.Parameters.Add(cbPort.Items[i]);
      cmd.Execute;
      list.LoadFromStream(cmd.Output);
      if list.Count>0 then begin
        StatusBar1.Panels[2].Text:=list[0];
        GUItext.Lines.Add(list[0]);
        GIMBALtext.Lines.Add(list[0]);
      end;
    end;
    if GIMBALtext.Lines.Count<1 then
      StatusBar1.Panels[2].Text:=errNoUSBport;
  finally
    cmd.Free;
    list.Free;
  end;
{$ENDIF}
end;

procedure TForm1.TakeScreenshot(filename: string);
var
    bild: TPortableNetworkGraphic;
    bmp: TBitMap;
    {$IFDEF WINDOWS}
      ScreenDC: HDC;
    {$ENDIF}

begin
  bild:=TPortableNetworkGraphic.Create;             {create PNG-picture}
  try
  {$IFDEF LINUX}
    bmp:=GetFormImage;                             {No more working with Windows}
    bild.Assign(bmp);
  {$ELSE}
    ScreenDC := GetDC(Handle);
    bild.LoadFromDevice(ScreenDC);                  {Get screenshot xyz.Handle}
    ReleaseDC(0, ScreenDC);
  {$ENDIF}
    bild.SaveToFile(filename);
  finally
    bild.Free;
    {$IFDEF LINUX}
      bmp.Free;
    {$ENDIF}
  end;
end;

procedure TForm1.acScreenshotExecute(Sender: TObject);
begin
  SaveDialog1.Title:=titScreenshot;
  SaveDialog1.FilterIndex:=3;
  SaveDialog1.FileName:=capScreenshot+FormatDateTime('yyyymmdd_hhnnss', now)+'.png';
  if SaveDialog1.Execute then
    TakeScreenshot(SaveDialog1.FileName);
end;

procedure TForm1.btnCenterClick(Sender: TObject);
begin
  knPanControl.Position:=2048;
end;

procedure TForm1.btnDisableNFZClick(Sender: TObject);
var
  msg: TMAVmessage;

begin
  lblOK.Caption:=tab1;
  if (vleSystem.Cells[1, 2]<>'') and UARTConnected then begin
    CreateMsg57(msg, vleSystem.Cells[1, 2], 2);
    SendUARTMessage(msg, LengthFixPartBC);
  end;
end;

procedure TForm1.btnEnableNFZClick(Sender: TObject);
var
  msg: TMAVmessage;

begin
  lblOK.Caption:=tab1;
  if (vleSystem.Cells[1, 2]<>'') and UARTConnected then begin
    CreateMsg57(msg, vleSystem.Cells[1, 2], 8);
    SendUARTMessage(msg, LengthFixPartBC);
  end;
end;

procedure SendYGCCommand(const CommandCode: byte);
var
  msg: TMAVmessage;

begin
  if UARTConnected then begin
    CreateYGCcommandMessage(msg, CommandCode);
    SendUARTMessage(msg, LengthFixPartFE);
  end;
end;

procedure TForm1.btnYawEncErsClick(Sender: TObject);
begin
  SendYGCCommand($0D);
end;

procedure TForm1.btnZeroPhaseErsClick(Sender: TObject);
begin
  SendYGCCommand($11);
end;

{FC sends some special commands to gimbal using the Pan mode.
 Commands:
  16 - ??
  24 - Gimbal calibration like from ST16
 }
procedure TForm1.btnGimbalCaliClick(Sender: TObject);
var
  msg: TMAVmessage;

begin
  if UARTConnected then begin
    CreateFCControl(msg, SequNumberTransmit, 24);
    if   SendUARTMessage(msg, LengthFixPartFE) then
      IncSequNo8(SequNumberTransmit);
  end;
end;

procedure TForm1.btnAccEraseClick(Sender: TObject);
begin
  rgYGC_Type.ItemIndex:=2;
  SendYGCCommand($13);
end;

procedure TForm1.btnFrontErsClick(Sender: TObject);
begin
  rgYGC_Type.ItemIndex:=1;
  SendYGCCommand($15);
end;

procedure TForm1.btnGeoFenceClick(Sender: TObject);
var
  msg: TMAVmessage;

begin
  CreateGUI_PARAM_SET(msg, pGeoFence, single(speGeoFence.Value));
  SendUARTMessage(msg, LengthFixPartBC);
end;

procedure TForm1.btnHeightLimitClick(Sender: TObject);
var
  msg: TMAVmessage;

begin
  CreateGUI_PARAM_SET(msg, pHeightLimit, single(speHeightLimit.Value));
  SendUARTMessage(msg, LengthFixPartBC);
end;

procedure TForm1.btnYawEncCaliClick(Sender: TObject);
begin
  SendYGCCommand($0C);
end;

procedure TForm1.btnPreFrontCaliClick(Sender: TObject);
begin
  SendYGCCommand($0F);
end;

procedure TForm1.btnTurnAllClick(Sender: TObject);
var
  MotorCommand: TCommandLong;
  msg: TMAVmessage;

begin
  if UARTconnected then begin
    MotorCommand:=Default(TCommandLong);
    MotorCommand.commandID:=209;
    MotorCommand.params[0]:=255;                       {Motor ID; 255 for all}
    MotorCommand.params[2]:=29;                        {RPM}
    if cbHighRPM.Checked then
      MotorCommand.params[2]:=highRPM;
    MotorCommand.params[3]:=2500;                      {Duration}
    CreateGUI_COMMAND_LONG(msg, motorcommand);
    SendUARTMessage(msg, LengthFixPartBC);
  end;
end;

procedure TForm1.btnZeroPhaseCaliClick(Sender: TObject);
begin
  SendYGCCommand($10);
end;

procedure TForm1.btnAccCaliClick(Sender: TObject);
begin
  SendYGCCommand($12);
end;

procedure TForm1.btnVersionClick(Sender: TObject);
begin
  SendYGCCommand($18);
end;

procedure TForm1.cbPortDblClick(Sender: TObject);
begin
  acScanPortsExecute(self);
end;

procedure TForm1.FormActivate(Sender: TObject);
begin
  StopAllTimer;
  knPanControl.Position:=2048;
  if rgYGC_Type.ItemIndex=4 then       {Do not remember Channel_data, it's rare}
    rgYGC_Type.ItemIndex:=0;
  acScanPortsExecute(self);
  btnConnect.SetFocus;
end;

procedure TForm1.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  DisconnectUART;
end;

procedure TForm1.knPanControlChange(Sender: TObject; AValue: Longint);
begin
  lblPanControl.Caption:=IntToStr(InvertPanControlPosition(knPanControl.Position));
end;

{Up to now I had the impression that the motor numbering is counterclockwise.
 This was also my interpretation of the motor error codes in Motor_status in
 flight logs. Also the error beep code support this view.

The motor numbers in the COMMAND_LONG message clockwise starting with 1 front right.
This means the motor numbers in command message are wrong assigned? }

procedure TForm1.picMotorsMouseDown(Sender: TObject; Button: TMouseButton;
                                    Shift: TShiftState; X, Y: Integer);
var
  hpos, vpos: byte;
  MotorCommand: TCommandLong;
  msg: TMAVmessage;

begin
  MotorCommand:=Default(TCommandLong);
  MotorCommand.commandID:=209;
  MotorCommand.params[2]:=29;                          {RPM}
  if cbHighRPM.Checked then
    MotorCommand.params[2]:=highRPM;
  MotorCommand.params[3]:=1000;                        {Duration}

  if x>240 then vpos:=4 else
    if x>162 then vpos:=3 else
      if x>90 then vpos:=2 else
        vpos:=1;
  if y>170 then hpos:=3 else
    if y>70 then hpos:=2 else
      hpos:=1;
  if hpos=1 then begin
    if (vpos=1) or (vpos=2) then begin
      StatusBar1.Panels[2].Text:=rsMotor+' 1 '+rsSelected+rsACW;
      MotorCommand.params[0]:=6;                       {Numbering CCW used}
    end;
    if (vpos=3) or (vpos=4) then begin
      StatusBar1.Panels[2].Text:=rsMotor+' 6 '+rsSelected+rsBCCW;
      MotorCommand.params[0]:=1;
    end;
  end else begin
    if hpos=2 then begin
      if vpos=1 then begin
        StatusBar1.Panels[2].Text:=rsMotor+' 2 '+rsSelected+rsBCCW;
        MotorCommand.params[0]:=5;
      end;
      if vpos=4 then begin
        StatusBar1.Panels[2].Text:=rsMotor+' 5 '+rsSelected+rsACW;
        MotorCommand.params[0]:=2;
      end;
    end else begin
      if hpos=3 then begin
        if (vpos=1) or (vpos=2) then begin
          StatusBar1.Panels[2].Text:=rsMotor+' 3'+rsSelected+rsACW;
          MotorCommand.params[0]:=4;
        end;
        if (vpos=3) or (vpos=4) then begin
          StatusBar1.Panels[2].Text:=rsMotor+' 4 '+rsSelected+rsBCCW;
          MotorCommand.params[0]:=3;
        end;
      end;
    end;
  end;

  if btnTurnAll.Enabled and UARTconnected then begin
    CreateGUI_COMMAND_LONG(msg, motorcommand);
    SendUARTMessage(msg, LengthFixPartBC);
  end;
end;

procedure TForm1.rgAddValueClick(Sender: TObject);
begin
  AddValueLineSeries1.Clear;
end;

procedure TForm1.rgYGC_TypeClick(Sender: TObject);
begin
  case rgYGC_Type.ItemIndex of
    0: WriteHeader_GYRO_POWER;
    1: WriteHeader_EULER_ANGLE;
    2: WriteHeader_ACC;
    3: WriteHeader_TEMP_DIFF;
    4: WriteHeader_Channel_data;
  end;
end;

procedure TForm1.timerFCHeartbeatTimer(Sender: TObject);
var
  msg: TMAVmessage;

begin
  if UARTConnected then begin
    CreateFCHeartBeat(msg, SequNumberTransmit);
    if SendUARTMessage(msg, LengthFixPartFE) then
      IncSequNo8(SequNumberTransmit);
  end;
  NumberMessagesInStatusBar;
end;

procedure TForm1.timerGUITimer(Sender: TObject);
var
  msg: TMAVmessage;

begin
  if UARTConnected then begin
    CreateGUIheartbeat(msg);
    SendUARTMessage(msg, LengthFixPartBC);

// Send empty messages necessary? Semms not.
(*    CreateGUI_SYS_STATUS(msg);
    SendUARTMessage(msg, LengthFixPartBC);
    CreateGUIemptyMsg(msg, 32, 28);                {LOCAL_POSITION_NED}
    SendUARTMessage(msg, LengthFixPartBC);
    CreateGUIemptyMsg(msg, 30, 28);                {ATTITUDE}
    SendUARTMessage(msg, LengthFixPartBC); *)

    NumberMessagesInStatusBar;
  end;
  timerSensors.Enabled:=true;
end;

procedure TForm1.timerSensorsTimer(Sender: TObject);
begin
  ResetSensorsStatus;
end;

procedure TForm1.timerTelemetryTimer(Sender: TObject);
var
  msg: TMAVmessage;

begin
  if UARTConnected then begin
    CreateFCTelemetry5GHz(msg, SequNumberTransmit);
    if  SendUARTMessage(msg, LengthFixPartFE) then
      IncSequNo8(SequNumberTransmit);
  end;
end;

procedure TForm1.timerYGCcommandLongTimer(Sender: TObject);
var
  msg: TMAVmessage;
  i: byte;

begin
  if UARTConnected then begin
    for i:= 1 to 5 do begin
      CreateYGCcommandMessageLong(msg, i);
      SendUARTMessage(msg, LengthFixPartFE);
    end;
  end;
end;

procedure TForm1.timerFCCommandTimer(Sender: TObject);
var
  msg: TMAVmessage;

begin
  if UARTConnected then begin
    CreateFCControl(msg, SequNumberTransmit);
    if   SendUARTMessage(msg, LengthFixPartFE) then
      IncSequNo8(SequNumberTransmit);
  end;
end;

// Charts

procedure TForm1.CreateSatSNRBarChart(const sats: TGPSdata);
var
  i, NumSatsInUse, NumSatsVisible: integer;
  IndicatorColor: TColor;

begin
  ChartSatSNR.DisableRedrawing;
  BarSatSNR.Clear;
  NumSatsInUse:=0;
  ChartSatSNR.Title.Visible:=true;
  try
    for i:=0 to MAVsatCount do begin
      if (sats.sat_used[i]<>0) then begin
        IndicatorColor:=clSatUsed;
        inc(NumSatsInUse);
      end else
        IndicatorColor:=clSatVisible;
      SatSNRBarSource.Add(i, sats.sat_snr[i], 'PRN'+IntToStr(sats.sat_prn[i]), IndicatorColor);
    end;
    NumSatsVisible:=sats.sats_visible;
    if NumSatsVisible=max8 then
      NumSatsVisible:=0;
    ChartSatSNR.Title.Text[0]:=IntToStr(NumSatsVisible)+tab1+rsVisible+tab2+
                               IntToStr(NumSatsInUse)+tab1+rsInUse;
    if (NumSatsInUse>0) and (gbPosition.Color<>clSatUsed) then
      gbPosition.Color:=clSatUsed;
    if NumSatsInUse=0 then
      gbPosition.Color:=clSensorOK;
  finally
    ChartSatSNR.EnableRedrawing;
    ChartSatSNR.Repaint;
  end;
end;

procedure TForm1.CreateSatPolarDiagram(const sats: TGPSdata);
var
  azi, ele: single;
  IndicatorColor: TColor;
  i: integer;

begin
  SatPolar.DisableRedrawing;
  try
    SatPolarSeries.Clear;
    for i:=0 to MAVsatCount do begin
      if (sats.sat_used[i]<>0) then
        IndicatorColor:=clSatUsed
      else
        IndicatorColor:=clSatVisible;
      azi:=SatAzimuthToDeg(sats.sat_azimuth[i]);
      ele:=SatElevationToDeg(sats.sat_elevation[i]);
      SatPolarSource.Add(azi, ele, IntToStr(sats.sat_prn[i]), IndicatorColor);
    end;
    PreparePolarAxes(SatPolar, 90);  {Sat elevation 0: right on top of receiver, 90: on the horizon}
  finally
    SatPolar.EnableRedrawing;
    SatPolar.Repaint;
  end;
end;


//////////////////////// Polar coordinates diagram /////////////////////////////

{https://www.lazarusforum.de/viewtopic.php?p=66139#p66139
 wp_xyz

PreparePolarAxes rufst du auf, nachdem deine Daten geladen sind und du weißt,
wie groß der maximale Radiuswert ist.
Den brauchst du als Parameter AMax in dieser Prozedur (hier fix auf 90°).
Damit werden die x- und y-Achsen auf gleichen Wertebereich eingestellt und
insgesamt komplett ausgeblendet.

DrawPolarAxes wird im OnAfterDrawBackwall-Ereignis des Charts aufgerufen.
Zu diesem Zeitpunkt sind die Daten noch nicht ausgegeben - es würde sich auch
OnAfterPaint anbieten, aber damit würden die Achsenkreise über die
Datenkurven gezeichnet und in der hier gezeigten Implementierung komplett
übermalt, weil das Hintergrundrechteck mit eingefärbt wird.
DrawPolarAxes erhält als Parameter wieder den maximalen Radius und den
Abstand der Kreise. Die komplette Zeichenausgabe ist etwas ungewohnt
("Chart.Drawer"), weil TAChart eine Zwischenschicht für die Ausgabe
eingeführt hat, so dass man verschiedene Ausgabe"geräte"
(BGRABitmap, Vektorformate, Drucker) mit demselben Code ansprechen kann.
}

procedure TForm1.PreparePolarAxes(AChart: TChart; AMax: Double);
var
  ex: TDoubleRect;

begin
  ex.a.x:= -AMax;
  ex.a.y:= -AMax;
  ex.b.x:= AMax;
  ex.b.y:= AMax;
  with AChart do begin
    Extent.FixTo(ex);
    Proportional:=true;
    Frame.Visible:=false;
    with LeftAxis do begin
      AxisPen.Visible:=false;
      Grid.Visible:=false;
      PositionUnits:=cuGraph;
      Marks.Visible:=false;
    end;
    with BottomAxis do begin
      AxisPen.Visible:=false;
      Grid.Visible:=false;
      PositionUnits:=cuGraph;
      Marks.Visible:=false;
    end;
  end;
end;

procedure TForm1.DrawPolarAxes(AChart: TChart; AMax, ADelta: Double; MeasurementUnit: string);
var
  xRadius, theta: Double;
  P1, P2: TPoint;
  i, h, w: Integer;
  AxisLabels: string;

begin
  with AChart do begin
// Background
    Drawer.SetBrushParams(bsSolid, Color);
    Drawer.FillRect(0, 0, Width, Height);

// Radial lines for direction
    Drawer.SetBrushParams(bsClear, clNone);
    Drawer.SetPenParams(psDot, clGray);
    for i:=0 to 5 do begin
      theta:=i * pi/6;
      P1:=GraphToImage(DoublePoint(AMax*sin(theta), AMax*cos(theta)));
      P2:=GraphToImage(DoublePoint(-AMax*sin(theta), -AMax*cos(theta)));
      Drawer.MoveTo(P1);
      Drawer.Lineto(P2);
    end;

// Circles
    xRadius:=ADelta;
    while xRadius <= AMax do begin
      P1:=GraphToImage(DoublePoint(-xRadius, -xRadius));
      P2:=GraphToImage(DoublePoint(+xRadius, +xRadius));
      Drawer.SetPenParams(psDot, clGray);
      Drawer.SetBrushParams(bsClear, clNone);
      Drawer.Ellipse(P1.x, P1.y, P2.x, P2.y);
      xRadius:=xRadius + ADelta;
    end;

// Axis labels
    Drawer.Font:=BottomAxis.Marks.LabelFont;
    h:=Drawer.TextExtent('0').y;
    xRadius:=0;
    while xRadius <= AMax do begin
      AxisLabels := FloatToStr(xRadius)+MeasurementUnit;
      w:=Drawer.TextExtent(AxisLabels).x;
      P1:=GraphToImage(DoublePoint(0, xRadius));
      Drawer.TextOut.Pos(P1.X - w div 2, P1.y - h div 2).Text(AxisLabels).Done;
      xRadius:=xRadius + ADelta;
    end;
  end;
end;

{DrawPolarAxes is called in the OnAfterDrawBackwall event of the chart.
 OnAfterPaint would also be an option, but this would draw the axis circles over the
 data curves because the background rectangle is also colored.}

procedure TForm1.SatPolarAfterDrawBackWall(ASender: TChart; ACanvas: TCanvas;
  const ARect: TRect);
begin
  DrawPolarAxes(ASender, 90, 15, '°');
end;

procedure TForm1.PrepareSatPolarDiagram;               {My settings for the polar diagram}
begin
  with SatPolarSeries do begin
    Source:=SatPolarSource;
    Marks.Style:=smsLabel;
    LinePen.Style:=psClear;
    ShowPoints:=true;
    Marks.LabelBrush.Color:=clPolarLabel;
    Pointer.Style:=psHexagon;
    Pointer.HorizSize:=PolarSatSize;
    Pointer.VertSize:=PolarSatSize;
    Pointer.Pen.Style:=psClear;
    Pointer.Visible:=true;
  end;
end;

procedure TForm1.PrepareSatSNRBarChart;                {My settings for the SNR chart}
begin
  with ChartSatSNR do begin                            {for the whole chart}
    Title.Visible:=false;
    LeftAxis.Title.Caption:=capSatSNR+' [db]';
    LeftAxis.Title.Visible:=true;
    BottomAxis.Marks.Source:=SatSNRBarSource;
    BottomAxis.Marks.Style:=smsLabel;
    BottomAxis.Grid.Visible:=false;
    BottomAxis.Marks.LabelFont.Orientation := 900;     {gedreht}
  end;

  with BarSatSNR do begin                              {For the bar serie}
    Source:=SatSNRBarSource;
    SeriesColor:=clSatUsed;
  end;
end;

end.

