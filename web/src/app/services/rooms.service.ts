import { Injectable } from "@angular/core";
import { HttpClient } from "@angular/common/http";

export interface CommonResponse {
  readonly response: string;
}
export interface IRoomData {
  readonly time: string;
  readonly alarm: boolean;
  readonly idBedroom: string;
  readonly idResidence: string;
  readonly idTenant: string;
  readonly lightLevel: number;
  readonly temperature: number;
  readonly humidity: number;
  readonly airQuality: number;
  readonly presence: boolean;
}

export interface Series {
  name: string;
  columns: string[];
  values: any[][];
}

export interface Result {
  statement_id: number;
  series: Series[];
}

export interface ResultsObject {
  results: Result[];
}

export interface IRoomEmergency {
  readonly time: string;
  readonly alarm: boolean;
  readonly idBedroom: string;
  readonly idResidence: string;
  readonly idTenant: string;
}

export interface IRoomDto {
  readonly rooms: IRoomData[];
}

export interface IRoomEmergencyDto {
  readonly rooms: IRoomEmergency[];
}

const NODE_RED_IP = "http://localhost:1880/";

@Injectable()
export class RoomsService {
  constructor(private http: HttpClient) {}

  async getRooms(idTenant: string, idResidence: string): Promise<IRoomDto> {
    const returnedObject = await this.http
      .get<ResultsObject>(
        NODE_RED_IP + "api/rooms/" + idTenant + "/" + idResidence
      )
      .toPromise();

    let rooms: IRoomData[] = this.resultsObjectToDto(returnedObject);

    rooms = rooms.sort(function (x, y) {
      return new Date(x.time).getTime() - new Date(y.time).getTime();
    });

    const finalRooms: IRoomData[] = [];
    for (const room of rooms) {
      const existentRoom = finalRooms.find(
        (r) =>
          r.idBedroom === room.idBedroom &&
          r.idResidence === room.idResidence &&
          r.idTenant === room.idTenant
      );

      if (!existentRoom) {
        finalRooms.push(room);
      } else {
        for (const key in room) {
          if (!existentRoom[key]) {
            existentRoom[key] = room[key];
          }
        }
      }
    }

    return { rooms: finalRooms };
  }

  async getEmergency(
    idTenant: string,
    idResidence: string
  ): Promise<IRoomEmergencyDto> {
    const returnedObject = await this.http
      .get<ResultsObject>(
        NODE_RED_IP + "api/rooms/emergency/" + idTenant + "/" + idResidence
      )
      .toPromise();

    let rooms: IRoomEmergency[] = this.resultsObjectToDto(returnedObject);
    let timeNow: number = new Date().getTime();
    rooms = rooms.filter((e) => {
      return timeNow - new Date(e.time).getTime() < 2500
    });

    return { rooms };
  }

  resultsObjectToDto(resultObject: ResultsObject): any[] {
    if (!resultObject) {
      return null;
    }
    let colums: string[] = [];
    const allValues: any[] = [];

    for (const result of resultObject.results) {
      for (const series of result.series) {
        colums = series.columns;
        for (const values of series.values) {
          allValues.push(values);
        }
      }
    }

    const mergedObjects: any[] = [];
    for (const values of allValues) {
      let objectToAdd: any = {};
      for (let i = 0; i < values.length; i++) {
        objectToAdd[colums[i]] = values[i];
      }
      mergedObjects.push(objectToAdd);
    }
    return mergedObjects;
  }
}
