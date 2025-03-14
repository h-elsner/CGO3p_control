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
  lclintf, lcltype, Buttons, ActnList, Process, XMLPropStorage, ComCtrls, Grids,
  ValEdit, TAGraph, TASeries, TAChartUtils, synaser, MKnob,
  clipbrd, Menus, mav_def, mav_msg, Types;

type

  { TForm1 }

  TForm1 = class(TForm)
    acConnect: TAction;
    acClose: TAction;
    acDisconnect: TAction;
    acScanPorts: TAction;
    acDeleteText: TAction;
    acSaveText: TAction;
    ActionList1: TActionList;
    btnTempCali: TButton;
    btnGimbalCali: TButton;
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
    btnReboot: TButton;
    btnTempErs: TButton;
    cbPort: TComboBox;
    cbRecord: TCheckBox;
    cbSpeed: TComboBox;
    Image1: TImage;
    ImageList1: TImageList;
    gridVarious: TStringGrid;
    knPanControl: TmKnob;
    lblCameraType: TLabel;
    lblTempCali: TLabel;
    lblWarning: TLabel;
    lblSerial: TLabel;
    lblSerialNo: TLabel;
    lblPowerCycle: TLabel;
    lblGimbalBootTime: TLabel;
    lblBootTime: TLabel;
    lblGimbalVersion: TLabel;
    lblPanControl: TLabel;
    GIMBALtext: TMemo;
    mnClear: TMenuItem;
    mnSaveText: TMenuItem;
    Separator1: TMenuItem;
    mnPorts: TMenuItem;
    panelTempCali: TPanel;
    pcMain: TPageControl;
    panelRight: TPanel;
    panelYGCTop: TPanel;
    mnText: TPopupMenu;
    rgYGC_Type: TRadioGroup;
    rgPanMode: TRadioGroup;
    rgTiltMode: TRadioGroup;
    SaveDialog1: TSaveDialog;
    StatusBar1: TStatusBar;
    gridStatus: TStringGrid;
    gridAttitude: TStringGrid;
    timerYGCcommandLong: TTimer;
    timerFCCommand: TTimer;
    timerFCHeartbeat: TTimer;
    tbTiltControl: TTrackBar;
    tsFC: TTabSheet;
    tsYGC: TTabSheet;
    upperPanel: TPanel;
    XMLPropStorage1: TXMLPropStorage;

    procedure acCloseExecute(Sender: TObject);
    procedure acConnectExecute(Sender: TObject);
    procedure acDeleteTextExecute(Sender: TObject);
    procedure acDisconnectExecute(Sender: TObject);
    procedure acSaveTextExecute(Sender: TObject);
    procedure acScanPortsExecute(Sender: TObject);
    procedure btnAccCaliClick(Sender: TObject);
    procedure btnAccEraseClick(Sender: TObject);
    procedure btnCenterClick(Sender: TObject);
    procedure btnFrontErsClick(Sender: TObject);
    procedure btnPreFrontCaliClick(Sender: TObject);
    procedure btnRebootClick(Sender: TObject);
    procedure btnTempCaliClick(Sender: TObject);
    procedure btnTempErsClick(Sender: TObject);
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
    procedure Image1Click(Sender: TObject);
    procedure knPanControlChange(Sender: TObject; AValue: Longint);
    procedure pcMainChange(Sender: TObject);
    procedure rgYGC_TypeClick(Sender: TObject);
    procedure timerFCCommandTimer(Sender: TObject);
    procedure timerFCHeartbeatTimer(Sender: TObject);
    procedure timerYGCcommandLongTimer(Sender: TObject);


  private
    procedure StopAllTimer;
    procedure CreateFCControl(var msg: TMavMessage; SequenceNumber: byte; const command: uint16=$FFFF);
    function PanModeToInt: uint16;
    function TiltModeToInt: uint16;
    procedure GridPrepare(var grid: TStringGrid; const NumRows: byte);
    procedure WriteHeader_STATUS;
    procedure WriteHeader_GYRO_POWER;
    procedure WriteHeader_EULER_ANGLE;
    procedure WriteHeader_ACC;
    procedure WriteHeader_TEMP_DIFF;
    procedure WriteHeader_Channel_data;
    procedure ClearMessageTables;
    procedure FillCharts;
    procedure GUIsetCaptionsAndHints;
  public
    procedure ReadMessage_FE(var msg: TMAVmessage);
    procedure ReadGimbalPosition(msg: TMAVmessage);
    procedure RecordMessage(msg: TMAVmessage; list: TStringList; LengthFixPart: byte);
    procedure ActAsFlightController(var msg: TMAVmessage; list: TStringList);
    procedure ActAsGimbalChecker(var msg: TMAVmessage; list: TStringList);
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

  {$I CGO3tool_en.inc}

var
  Form1: TForm1;
  UART: TBlockSerial;
  UARTConnected, SerialNumberFound: boolean;
  starttime: UInt64;
  SequNumberTransmit: byte;
  MessagesSent, MessagesReceived: integer;
  pan, roll, tilt, voltage: uint16;

const
  AppVersion='V1.2 2024-03-11';
  linkLazarus='https://www.lazarus-ide.org/';

  tab1=' ';
  tab2='  ';

  maxPorts=10;
  timeout=100;
  wait=5;

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
  WriteHeader_STATUS;
  WriteHeader_GYRO_POWER;
  GUIsetCaptionsAndHints;

  gridAttitude.Cells[0, 0]:='Gimbal';
  gridAttitude.Cells[0, 1]:='Tilt';
  gridAttitude.Cells[0, 2]:='Roll';
  gridAttitude.Cells[0, 3]:='Pan';
  gridAttitude.Cells[1, 0]:=rsValue;

  if ParamStr(1)='-tc' then
    panelTempCali.Visible:=true
  else
    panelTempCali.Visible:=false;
end;

procedure TForm1.GUIsetCaptionsAndHints;
begin
  Caption:=Application.Title+tab2+AppVersion;
  cbSpeed.Text:=IntToStr(defaultbaudCGO3);
  cbSpeed.Hint:=hntSpeed;
  cbPort.Hint:=hntPort;
  acScanPorts.Caption:=capPort;
  acScanPorts.Hint:=hntPort;
  acDeleteText.Caption:=capDeleteText;
  acDeleteText.Hint:=hntDeleteText;
  acSaveText.Caption:=capSaveText;
  acSaveText.Hint:=hntSavetext;
  acConnect.Caption:=capConnect;
  acConnect.Hint:=hntConnect;
  acDisConnect.Caption:=capDisConnect;
  acDisConnect.Hint:=hntDisConnect;
  acClose.Caption:=capClose;
  lblWarning.Caption:=capWarning;
  lblPowerCycle.Hint:=hntPowerCycle;

  btnGimbalCali.Caption:=capGimbalCali;
  btnGimbalCali.Hint:=hntGimbalCali;
  btnVersion.Caption:=capVersion;
  btnVersion.Hint:=hntVersion;
  btnCenter.Caption:=capCenter;
  btnCenter.Hint:=hntCenter;
  btnReboot.Caption:=capReboot;
  btnReboot.Hint:=hntReboot;

  lblBootTime.Caption:=capFCtime;
  lblBootTime.Hint:=hntBoottime;
  lblSerialNo.Caption:=capSerialNo;

  btnClose.Hint:=hntClose;
  StatusBar1.Hint:=hntStatusBar;
  rgYGC_Type.Caption:=capYGC_Type;
  rgYGC_Type.Hint:=hntYGC_Type;
  panelRight.Hint:=hntPanelRight;
  tbTiltControl.Hint:=hntTiltControl;
  rgTiltMode.Hint:=hntTiltMode;
  rgPanMode.Hint:=hntPanMode;
  knPanControl.Hint:=hntPanControl;

  cbRecord.Caption:=capRecord;
  cbRecord.Hint:=hntRecord;

  tsYGC.Caption:=captsYGC;
  tsFC.Caption:=captsFC;
  panelTempCali.Hint:=hntTempCali;
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
  gridStatus.Cells[1, 0]:=rsvalue;
  gridVarious.Cells[1, 0]:=rsValue;
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
begin                                              {Textfarbe abh. vom Hintergrund}
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
  result:=2048;
  case rgTiltMode.ItemIndex of
    0: result:=2184;
    1: result:=3412;
  end;
end;

function TForm1.PanModeToInt: uint16;
begin
  result:=2048;
  case rgPanMode.ItemIndex of
    0: result:=683;
    1: result:=1502;
    2: result:=3412;
    3: result:=1433;                                   {Teammode}
    4: result:=2048;                                   {Neutral - sollte eigentlich nicht auftrete }
  end;
end;

procedure TForm1.StopAllTimer;
begin
  timerFCHeartbeat.Enabled:=false;
  timerYGCcommandLong.Enabled:=false;
  timerFCCommand.Enabled:=false;
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
  voltage:=0;                                          {mV}
  SequNumberTransmit:=0;
  MessagesSent:=0;
  MessagesReceived:=0;
  SerialNumberFound:=false;
  starttime:=GetTickCount64;
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
  SetCRC(msg, LengthFixPartFE);
end;

function ConnectUART(port, speed: string): string;
begin
  result:='';
  if UARTconnected then
    exit;
  UART:=TBlockSerial.Create;
  {$IFDEF LINUX}
    UART.LinuxLock:=false;
  {$ENDIF}
  UART.Connect(port);
  sleep(200);
  UART.Config(StrToIntDef(speed, defaultbaudCGO3), 8, 'N', SB1, false, false); {Config default 115200 baud, 8N1}
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
  i: byte;

begin
  csvlist:=TStringList.Create;
  try
    msg:=Default(TMAVmessage);
    SetStartValuesForGlobelVariables;
    for i:=1 to 3 do
      gridAttitude.Cells[1, i]:='';
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
          ActAsGimbalChecker(msg, csvlist);

      NumberMessagesInStatusBar;
      SaveDialog1.FilterIndex:=1;
      SaveDialog1.FileName:='FEmessages_'+FormatDateTime('yyyymmdd_hhnnss', now)+'.csv';
      if cbRecord.Checked and (csvlist.Count>1) and SaveDialog1.Execute then begin
        csvlist.SaveToFile(SaveDialog1.FileName);
        StatusBar1.Panels[2].Text:=SaveDialog1.FileName+rsSaved;
      end;
    end;
  finally
    csvlist.Free;
  end;
end;

procedure TForm1.acDeleteTextExecute(Sender: TObject);
begin
  gimbalText.Lines.Clear;
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

procedure TForm1.FillCharts;
begin
  if tilt<>0 then begin
    gridAttitude.Cells[1, 1]:=FormatFloat(floatformat2, GimbalAngleToDegree(tilt));
    gridAttitude.Cells[1, 2]:=FormatFloat(floatformat2, GimbalAngleToDegree(roll));
    gridAttitude.Cells[1, 3]:=FormatFloat(floatformat2, GimbalPanToDegree(pan));
  end;
end;

function FormatBootTime(const data: TGPSdata): string;
begin
  result:=FormatDateTime(timezzz, data.boottime);
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
        if cbRecord.Checked then
          RecordMessage(msg, list, LengthFixPartFE);
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
  for i:=2 to len+LengthFixPartFE+1 do begin
    msg.msgbytes[i]:=UART.RecvByte(timeout);
  end;
  if CheckCRC16MAV(msg, LengthFixPartFE, 1, true) then begin
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
  StatusBar1.Panels[2].Text:=rsDisconnected;
end;

procedure TForm1.acSaveTextExecute(Sender: TObject);
begin
  SaveDialog1.Title:=titSaveText;
  SaveDialog1.FilterIndex:=2;
  SaveDialog1.FileName:=FormatDateTime('yyyymmdd_hhnnss', now)+'.txt';
  if SaveDialog1.Execute then
    GIMBALtext.Lines.SaveToFile(SaveDialog1.FileName);
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
  GimbalText.Lines.Clear;
  cbPort.Items.CommaText:=GetSerialPortNames;
  if cbPort.Items.Count>0 then begin
    cbPort.Text:=cbPort.Items[cbPort.Items.Count-1];
    for i:=0 to  cbPort.Items.Count-1 do begin
      GIMBALtext.Lines.Add(cbPort.Items[i]);
    end;
    StatusBar1.Panels[2].Text:=cbPort.Items[cbPort.Items.Count-1];
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

procedure TForm1.btnCenterClick(Sender: TObject);
begin
  knPanControl.Position:=2048;
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
      Inc(SequNumberTransmit);
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

procedure TForm1.btnYawEncCaliClick(Sender: TObject);
begin
  SendYGCCommand($0C);
end;

procedure TForm1.btnPreFrontCaliClick(Sender: TObject);
begin
  SendYGCCommand($0F);
end;

procedure TForm1.btnRebootClick(Sender: TObject);
begin
  SendYGCCommand($19);
end;

procedure TForm1.btnTempCaliClick(Sender: TObject);
begin
  SendYGCCommand($09);
end;

procedure TForm1.btnTempErsClick(Sender: TObject);
begin
  SendYGCCommand($0A);
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
  if not UARTconnected then begin
    StopAllTimer;
    knPanControl.Position:=2048;
    if rgYGC_Type.ItemIndex=4 then       {Do not remember Channel_data, it's rare}
      rgYGC_Type.ItemIndex:=0;
    acScanPortsExecute(self);
    btnConnect.SetFocus;
  end;
end;

procedure TForm1.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  DisconnectUART;
end;

procedure TForm1.knPanControlChange(Sender: TObject; AValue: Longint);
begin
  lblPanControl.Caption:=IntToStr(InvertPanControlPosition(knPanControl.Position));
end;

procedure TForm1.pcMainChange(Sender: TObject);
begin
  acDisconnectExecute(self);
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
      Inc(SequNumberTransmit);
  end;
  NumberMessagesInStatusBar;
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
      Inc(SequNumberTransmit);
  end;
end;

end.

