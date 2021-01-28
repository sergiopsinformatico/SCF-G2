import { Injectable } from "@angular/core";
import { HttpClient } from "@angular/common/http";

export interface IRoom {
  readonly nodeName: string;
  readonly nodeId: string;
  light: number;
  temperature: number;
  humidity: number;
  airQuality: number;
  presence: boolean;
  emergency: boolean;
}

export interface IRoomDto {
  readonly retirement: string;
  readonly rooms: IRoom[];
}

const NODE_RED_IP = "http://192.168.1.156:1880/";

@Injectable()
export class RoomsService {
  constructor(private http: HttpClient) {}

  async getRooms(retirement: string): Promise<IRoomDto> {
    return this.http
      .get<IRoomDto>(NODE_RED_IP + "api/rooms/" + retirement)
      .toPromise();
  }
}
