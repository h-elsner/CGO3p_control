unit CGO3read_main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, ExtCtrls, StdCtrls,
  Buttons, ActnList, Menus, Process, XMLPropStorage, ComCtrls, Grids, TAGraph,
  TASeries, synaser, MKnob, mav_def, mav_msg, Types;

type

  { TForm1 }

  TForm1 = class(TForm)
    acConnect: TAction;
    acClose: TAction;
    acDisconnect: TAction;
    acScanPorts: TAction;
    ActionList1: TActionList;
    btnDisconnect: TBitBtn;
    btnClose: TBitBtn;
    btnConnect: TBitBtn;
    btnCenter: TButton;
    btnVersion: TButton;
    cbPort: TComboBox;
    cbRecord: TCheckBox;
    cbSpeed: TComboBox;
    cbTelemetry: TCheckBox;
    chPanLineSeries1: TLineSeries;
    chRoll: TChart;
    chRollLineSeries1: TLineSeries;
    chTilt: TChart;
    chPan: TChart;
    chTiltLineSeries1: TLineSeries;
    knPanControl: TmKnob;
    lblGimbalVersion: TLabel;
    lblPanControl: TLabel;
    Memo1: TMemo;
    PageControl: TPageControl;
    panelRight: TPanel;
    panelYGCTop: TPanel;
    rgPanMode: TRadioGroup;
    rgTiltMode: TRadioGroup;
    SaveDialog1: TSaveDialog;
    StatusBar1: TStatusBar;
    gridLeft: TStringGrid;
    timerYGCHeartbeat: TTimer;
    timerFCCommand: TTimer;
    timerTelemetry: TTimer;
    timerFCHeartbeat: TTimer;
    tbTiltControl: TTrackBar;
    tsFC: TTabSheet;
    tsYGC: TTabSheet;
    upperPanel: TPanel;
    XMLPropStorage1: TXMLPropStorage;
    procedure acCloseExecute(Sender: TObject);
    procedure acConnectExecute(Sender: TObject);
    procedure acDisconnectExecute(Sender: TObject);
    procedure acScanPortsExecute(Sender: TObject);
    procedure btnCenterClick(Sender: TObject);
    procedure btnVersionClick(Sender: TObject);
    procedure cbPortDblClick(Sender: TObject);
    procedure FormActivate(Sender: TObject);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure knPanControlChange(Sender: TObject; AValue: Longint);
    procedure timerFCCommandTimer(Sender: TObject);
    procedure timerFCHeartbeatTimer(Sender: TObject);
    procedure timerTelemetryTimer(Sender: TObject);
    procedure timerYGCHeartbeatTimer(Sender: TObject);
  private
    procedure StopAllTimer;
    procedure CreateFCControl(var msg: TMavMessage; SequenceNumber: byte);
    procedure CreateTelemetry5GHz(var msg: TMavMessage; SequenceNumber: byte);
    function PanModeToInt: uint16;
    function TiltModeToInt: uint16;
  public
    procedure ReadMessage(var msg: TMAVmessage);
    procedure ReadGimbalPosition(msg: TMAVmessage);
    procedure RecordMessage(msg: TMAVmessage; list: TStringList);
    procedure FillCharts;
    procedure ActAsFlightController(var msg: TMAVmessage; list: TStringList);
    procedure ActAsGimbalChecker(var msg: TMAVmessage; list: TStringList);
    procedure ReadYGCcameraMessages(msg: TMAVmessage);

    procedure CAM_SERIAL(msg: TMAVmessage);
    procedure TEXT_MESSAGE(msg: TMAVmessage);
  end;

var
  Form1: TForm1;
  UART: TBlockSerial;
  UARTConnected, ser: boolean;

  boottime: UInt64;
  SequNumberTransmit, SequNumberReceived: byte;
  MessagesSent, MessagesReceived: integer;
  pan, roll, tilt, voltage: uint16;


const
  AppName='Read CGO3 UART';
  AppVersion='V0.3 2024-12-29';

  tab1=' ';
  tab2='  ';

  maxPorts=10;
  timeout=100;
  defaultbaud=115200;

{$IFDEF WINDOWS}
  default_port='COM6';
{$ELSE}                                                {LINUX}
  default_port='/dev/ttyUSB0';
{$ENDIF}


implementation

{$R *.lfm}

{ TForm1 }

procedure TForm1.FormCreate(Sender: TObject);
begin
  Caption:=AppName+tab2+AppVersion;
  cbSpeed.Text:=IntToStr(defaultbaud);
  UARTconnected:=false;
  Memo1.Lines.Clear;
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
    3: result:=1433;      { Teammode}
    4: result:=2048;      { Neutral - sollte eigentlich nicht auftreten }
  end;
end;

procedure IncSequNo8(var no: byte);
begin
  if no<255 then
    no:=no+1
  else
    no:=0;
end;

procedure SetUInt16ToMsg(var msg: TMavMessage; const pos, value: uint16);
var
  v: uint16;

begin
  v:=value;
  msg.msgbytes[pos]:=v and $00FF;   {value low}
  v:=v shr 8;
  msg.msgbytes[pos+1]:=v and $00FF; {value high}
end;

procedure TForm1.StopAllTimer;
begin
  timerFCHeartbeat.Enabled:=false;
  timerYGCHeartbeat.Enabled:=false;
  timerTelemetry.Enabled:=false;
  timerFCCommand.Enabled:=false;
end;

procedure WriteCSVRawHeader(var list: TStringList);
var
  s: string;
  i: integer;

begin
  list.Clear;
  s:='Time';
  for i:=0 to 50 do
    s:=s+';'+Format('%.*d', [2, i]);
  list.Add(s);
end;

procedure SetStartValuesForGlobelVariables;
begin
  pan:=0;
  tilt:=0;
  roll:=0;
  voltage:=0;                         {[mV]}
  SequNumberTransmit:=0;
  SequNumberReceived:=0;
  MessagesSent:=0;
  MessagesReceived:=0;
  ser:=false;
  Boottime:=GetTickCount64;
end;

procedure CreateStandardPartMsg(var msg: TMAVmessage; const MsgLength: byte);
begin
  ClearMAVmessage(msg);
  msg.msgbytes[0]:=MagicFE;
  msg.msgbytes[1]:=MsgLength;
  msg.msglength:=msg.msgbytes[1];
  msg.msgbytes[3]:=1;                                  {SysId FC}
end;

procedure SetCRC(var msg: TMAVmessage);
var
  crc: uint16;

begin
  crc:=CRC16MAV(msg, LengthFixPartFE);
  SetUInt16ToMsg(msg, msg.msglength+LengthFixPartFE, crc);
  msg.valid:=true;
end;

procedure CreateFCHeartBeat(var msg: TMavMessage; SequenceNumber: byte);
begin
  CreateStandardPartMsg(msg, 5);
  msg.msgbytes[2]:=SequenceNumber;
  msg.msgbytes[12]:=1;
  SetCRC(msg);
end;

{ YGC_Type from gimbal
    1: 'GYRO_POWER'
    2: 'EULER_ANGLE'
    3: 'ACC'
    5: 'TEMP_DIFF'
    6: 'M_STATUS'
    $12: 'Undef 0x12 (18)   1Hz'    Ser-No?
    $FE: 'FW_Info'

 YGC_Command
    $14: 'Front_cali?'
    $18: 'Read_SWversion'
    $24: 'Heartbeat'        1Hz }

procedure CreateYGCcommandMessage(var msg: TMavMessage; const command: byte=$24);
begin
  CreateStandardPartMsg(msg, 2);
  msg.msgbytes[2]:=1;
  msg.msgbytes[3]:=YGCsysID;                            {SysId YGC}
  msg.msgbytes[5]:=2;                                   {to Gimbal}
  msg.msgbytes[7]:=2;                                   {MsgID}
  msg.msgbytes[8]:=command;                             {YGC_Type}
  msg.msgbytes[8]:=command;                             {YGC_Command}
  SetCRC(msg);
end;

procedure TForm1.CreateFCControl(var msg: TMavMessage; SequenceNumber: byte);
begin
  CreateStandardPartMsg(msg, 26);
  msg.msgbytes[2]:=SequenceNumber;
  msg.msgbytes[5]:=2;                                  {TargetId Gimbal}
  msg.msgbytes[7]:=1;                                  {MsgID}

  SetUInt16ToMsg(msg, 22, InvertPanControlPosition(knPanControl.Position));
  SetUInt16ToMsg(msg, 24, tbTiltControl.Position);
  SetUInt16ToMsg(msg, 26, 2048);
  SetUInt16ToMsg(msg, 28, PanModeToInt);
  SetUInt16ToMsg(msg, 30, TiltModeToInt);
  SetUInt16ToMsg(msg, 32, 500);
  SetCRC(msg);
end;

procedure TForm1.CreateTelemetry5GHz(var msg: TMavMessage; SequenceNumber: byte);
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

  SetCRC(msg);
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
  sleep(50);
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
      UARTConnected:=false;
    end;
  end;
end;

procedure TForm1.acConnectExecute(Sender: TObject);
var
  msg: TMAVmessage;
  csvlist: TStringList;

begin
  csvlist:=TStringList.Create;
  try
    SetStartValuesForGlobelVariables;
    chTiltLineSeries1.Clear;
    chPanLineSeries1.Clear;
    chRollLineSeries1.Clear;
    Memo1.Lines.Clear;
    lblGimbalVersion.Caption:='';
    StatusBar1.Panels[0].Text:='0';          {Sent messages}
    StatusBar1.Panels[1].Text:='0';          {Received messages}
    WriteCSVRawHeader(csvlist);
    StatusBar1.Panels[2].Text:=ConnectUART(cbPort.Text, cbSpeed.Text);

    if PageControl.ActivePage=tsFC then
      ActAsFlightController(msg, csvlist)
    else
      if PageControl.ActivePage=tsYGC then
        ActAsGimbalChecker(msg, csvlist);

    StatusBar1.Panels[0].Text:='S: '+IntToStr(MessagesSent);
    StatusBar1.Panels[1].Text:='R: '+IntToStr(MessagesReceived);
    if cbRecord.Checked and SaveDialog1.Execute then
      csvlist.SaveToFile(SaveDialog1.FileName);
  finally
    csvlist.Free;
  end;
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


procedure TForm1.CAM_SERIAL(msg: TMAVmessage);
var
  s: string;

begin
  if ser then
    exit;
  s:=GetCAM_SERIAL(msg);
  Memo1.Lines.Add(s);
  Memo1.Lines.Add('');
  Caption:=AppName+tab2+s;
  ser:=true;                          {Read serial number only once}
end;

procedure TForm1.TEXT_MESSAGE(msg: TMAVmessage);
var
  s: string;

begin
  s:=GetTEXT_MESSAGE(msg);
  Memo1.Lines.Add(s);
end;

procedure TForm1.ReadYGCcameraMessages(msg: TMAVmessage);
begin
  if (msg.msgid=2) and (msg.targetid=YGCsysID) then begin
    case msg.msgbytes[8] of             {YGC message type}
      $12: CAM_SERIAL(msg);
      $FE: TEXT_MESSAGE(msg);
    end;
  end;
end;

procedure TForm1.RecordMessage(msg: TMAVmessage; list: TStringList);
var
  s: string;
  i: integer;

begin
  s:=FormatFloat('0.000', (GetTickCount64-boottime)/1000);
  for i:=0 to msg.msglength+LengthFixPartFE+1 do begin
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

procedure TForm1.ActAsGimbalChecker(var msg: TMAVmessage; list: TStringList);
begin
  timerYGCHeartbeat.Enabled:=true;
  sleep(100);
  while (UART.LastError=0) and UARTConnected do begin
    if UART.CanRead(0) then begin
      ReadMessage(msg);
      if msg.valid then begin
        ReadYGCcameraMessages(msg);

        ReadGimbalPosition(msg);
        FillCharts;



        if cbRecord.Checked then
          RecordMessage(msg, list);
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
    timerTelemetry.Enabled:=true;           {Test telemetry only}
  end;
  while (UART.LastError=0) and UARTConnected do begin
    if UART.CanRead(0) then begin
      ReadMessage(msg);
      if msg.valid then begin
        ReadGimbalPosition(msg);
        if lblGimbalVersion.Caption='' then
          lblGimbalVersion.Caption:=GetGIMBAL_FW_VERSION(msg);
        FillCharts;
        if cbRecord.Checked then
          RecordMessage(msg, list);
        inc(MessagesReceived);
      end;
    end;
    Application.ProcessMessages;
  end;
end;

procedure TForm1.ReadMessage(var msg: TMAVmessage);
var
  b, len: byte;
  i: integer;

begin
  msg.valid:=false;
  repeat
    b:=UART.RecvByte(timeout);
  until (b=MagicFE) or (not UARTConnected);
  msg.msgbytes[0]:=b;
  len:=UART.RecvByte(timeout);
  msg.msgbytes[1]:=len;                     {Message lenght}
  msg.msglength:=len;
  b:=UART.RecvByte(timeout);
  msg.msgbytes[2]:=b;                       {Sequ number}
  b:=UART.RecvByte(timeout);
  if b>10 then
    exit;
  msg.msgbytes[3]:=b;                       {SysID}
  b:=UART.RecvByte(timeout);
  if b>0 then
    exit;
  msg.msgbytes[4]:=b;                       {CompID}
  b:=UART.RecvByte(timeout);
  msg.msgbytes[5]:=b;                       {TargetID}
  b:=UART.RecvByte(timeout);
  if b>0 then
    exit;
  msg.msgbytes[6]:=b;                       {SubTargetID}
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
  StatusBar1.Panels[2].Text:='Disconnected';
end;

procedure TForm1.acScanPortsExecute(Sender: TObject);
{$IFDEF LINUX}
var
  cmd: TProcess;
  list: TStringList;
{$ENDIF}

begin
{$IFDEF WINDOWS}
  cbPort.Text:='';
  cbPort.Items.Clear;
  cbPort.Items.CommaText:=GetSerialPortNames;
  cbPort.Text:=cbPort.Items[cbPort.Items.Count-1];
{$ENDIF}
{$IFDEF LINUX}
  cmd:=TProcess.Create(nil);
  list:=TStringList.Create;
  try
    cmd.Options:=cmd.Options+[poWaitOnExit, poUsePipes];
    cmd.Executable:='ls';
    cmd.Parameters.Clear;
    cmd.Parameters.Add('-l');
    cmd.Parameters.Add(cbPort.Text);
    cmd.Execute;
    list.LoadFromStream(cmd.Output);
    if list.Count>0 then
      StatusBar1.Panels[2].Text:=list[0]
    else
      StatusBar1.Panels[2].Text:='Port not available';
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

procedure TForm1.btnVersionClick(Sender: TObject);
var
  msg: TMAVmessage;

begin
  if UARTConnected then begin
    CreateYGCcommandMessage(msg, $18);
    if msg.valid then begin
      if UART.SendBuffer(@msg.msgbytes, msg.msglength+LengthFixPartFE+2)>LengthFixPartFE then
        inc(MessagesSent);
    end;
  end;
end;

procedure TForm1.cbPortDblClick(Sender: TObject);
begin
  acScanPortsExecute(self);
end;

procedure TForm1.FormActivate(Sender: TObject);
begin
  StopAllTimer;
  knPanControl.Position:=2048;
  acScanPortsExecute(self);
end;

procedure TForm1.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  DisconnectUART;
end;

procedure TForm1.knPanControlChange(Sender: TObject; AValue: Longint);
begin
  lblPanControl.Caption:=IntToStr(InvertPanControlPosition(knPanControl.Position));
end;

procedure TForm1.timerFCHeartbeatTimer(Sender: TObject);
var
  msg: TMAVmessage;

begin
  if UARTConnected then begin
    CreateFCHeartBeat(msg, SequNumberTransmit);
    if msg.valid then begin
      IncSequNo8(SequNumberTransmit);
      if UART.SendBuffer(@msg.msgbytes, msg.msglength+LengthFixPartFE+2)>LengthFixPartFE then
        inc(MessagesSent);
    end;
  end;
  StatusBar1.Panels[0].Text:='S: '+IntToStr(MessagesSent);
  StatusBar1.Panels[1].Text:='R: '+IntToStr(MessagesReceived);
end;

procedure TForm1.timerTelemetryTimer(Sender: TObject);
var
  msg: TMAVmessage;

begin
  if UARTConnected then begin
    CreateTelemetry5GHz(msg, SequNumberTransmit);
    if msg.valid then begin
      IncSequNo8(SequNumberTransmit);
      if UART.SendBuffer(@msg.msgbytes, msg.msglength+LengthFixPartFE+2)>LengthFixPartFE then
        inc(MessagesSent);
    end;
  end;
end;

procedure TForm1.timerYGCHeartbeatTimer(Sender: TObject);
var
  msg: TMAVmessage;

begin
  if UARTConnected then begin
    CreateYGCcommandMessage(msg);
    if msg.valid then begin
      if UART.SendBuffer(@msg.msgbytes, msg.msglength+LengthFixPartFE+2)>LengthFixPartFE then
        inc(MessagesSent);
    end;
  end;
  StatusBar1.Panels[0].Text:='S: '+IntToStr(MessagesSent);
  StatusBar1.Panels[1].Text:='R: '+IntToStr(MessagesReceived);
end;

procedure TForm1.timerFCCommandTimer(Sender: TObject);
var
  msg: TMAVmessage;

begin
  if UARTConnected then begin
    CreateFCControl(msg, SequNumberTransmit);
    if msg.valid then begin
      IncSequNo8(SequNumberTransmit);
      if UART.SendBuffer(@msg.msgbytes, msg.msglength+LengthFixPartFE+2)>LengthFixPartFE then
        inc(MessagesSent);
    end;
  end;
end;

end.

