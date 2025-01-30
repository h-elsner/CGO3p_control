{********************************************************}
{                                                        }
{       Common functions and definitions                 }
{                                                        }
{       Copyright (c) 2025    Helmut Elsner              }
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

*******************************************************************************)

unit msg57;

{$mode objfpc}{$H+}

interface

uses
  Classes, mav_def, mav_msg, MD5;

procedure CreateMsg57(var msg: TMAVmessage; const serial: shortstring; cmd: byte);

implementation

procedure CreateMsg57(var msg: TMAVmessage; const serial: shortstring; cmd: byte);
var
  HashStream: TMemoryStream;
  HashBuffer: array [0..15] of byte;
  i: byte;

begin
  HashStream:=TMemoryStream.Create;
  try
    HashBuffer:=MD5String(serial);
    HashStream.WriteBuffer(HashBuffer, 16);
    CreateStandardGUIMsg(msg, 18);
    msg.msgbytes[5]:=57;
    msg.msgbytes[6]:=cmd;
    for i:=0 to 15 do
      msg.msgbytes[i+LengthFixPartBC+2]:=HashBuffer[i];
    SetCRC_BC(msg);
  finally
    HashStream.Free;
  end;
end;

end.
