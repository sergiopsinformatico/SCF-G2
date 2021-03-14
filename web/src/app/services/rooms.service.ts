import { Injectable } from "@angular/core";
import { HttpClient } from "@angular/common/http";

export interface CommonResponse {
  readonly response: string;
}
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

const NODE_RED_IP = "http://localhost:1880/";

@Injectable()
export class RoomsService {
  constructor(private http: HttpClient) {}

  async getRooms(retirement: string): Promise<IRoomDto> {
    return this.http
      .get<IRoomDto>(NODE_RED_IP + "api/rooms/" + retirement)
      .toPromise();
  }

  async disableEmergency(
    retirement: string,
    room: string
  ): Promise<CommonResponse> {
    return this.http
      .post<CommonResponse>(
        NODE_RED_IP + "api/emergency/" + retirement + "/" + room,
        { retirement, room, emergencyStatus: "INACTIVE" }
      )
      .toPromise();
  }
}
