import { Injectable } from "@angular/core";
import { HttpClient } from "@angular/common/http";

export interface CommonResponse {
  readonly response: string;
}
export interface IRoomData {
  readonly nodeName: string;
  readonly nodeId: string;
  readonly light: number;
  readonly temperature: number;
  readonly humidity: number;
  readonly airQuality: number;
  readonly presence: boolean;
}

export interface IRoomEmergency {
  readonly nodeName: string;
  readonly nodeId: string;
  readonly emergency: boolean;
}

export interface IRoomDto {
  readonly retirement: string;
  readonly rooms: IRoomData[];
}

export interface IRoomEmergencyDto {
  readonly retirement: string;
  readonly rooms: IRoomEmergency[];
}

const NODE_RED_IP = "http://localhost:1880/";

@Injectable()
export class RoomsService {
  constructor(private http: HttpClient) {}

  async getRooms(retirement: string): Promise<IRoomDto> {
    return this.http
      .get<IRoomDto>(NODE_RED_IP + "api/rooms/" + retirement)
      .toPromise();
  }

  async getEmergency(retirement: string): Promise<IRoomEmergencyDto> {
    return this.http
      .get<IRoomEmergencyDto>(NODE_RED_IP + "api/rooms/" + retirement + "/emergency/500")
      .toPromise();
  }
}
