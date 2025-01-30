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
  mav_def;

procedure CreateMsg57(var msg: TMAVmessage; const serial: shortstring; cmd: byte);

implementation

procedure CreateMsg57(var msg: TMAVmessage; const serial: shortstring; cmd: byte);
begin
  msg.msgbytes[5]:=57;
  msg.msgbytes[6]:=cmd;
// doing this is not legal although I know how ;-)

  msg.valid:=false;
end;

end.
