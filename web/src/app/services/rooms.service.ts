import { Injectable } from "@angular/core";
import { HttpClient } from "@angular/common/http";
import { isDefined } from "@angular/compiler/src/util";

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

    if (!returnedObject) {
      return { rooms: [] };
    }

    let rooms: IRoomData[] = this.resultsObjectToDto(returnedObject);
    if (!rooms) {
      return { rooms: [] };
    }

    rooms = rooms.sort(function (x, y) {
      return new Date(y.time).getTime() - new Date(x.time).getTime();
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
          if (!isDefined(existentRoom[key])) {
            existentRoom[key] = room[key];
          }
        }
      }
    }

    return { rooms: finalRooms };
  }

  async getRoom(
    idTenant: string,
    idResidence: string,
    idBedroom: string
  ): Promise<IRoomDto> {
    const returnedObject = await this.http
      .get<ResultsObject>(
        NODE_RED_IP +
          "api/room/" +
          idTenant +
          "/" +
          idResidence +
          "/" +
          idBedroom
      )
      .toPromise();

    if (!returnedObject) {
      return { rooms: [] };
    }

    let rooms: IRoomData[] = this.resultsObjectToDto(returnedObject);
    if (!rooms) {
      return { rooms: [] };
    }

    rooms = rooms.sort(function (x, y) {
      return new Date(y.time).getTime() - new Date(x.time).getTime();
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
          if (!isDefined(existentRoom[key])) {
            existentRoom[key] = room[key];
          }
        }
      }
    }
    return { rooms: finalRooms };
  }

  async getEmergency(
    idTenant: string,
    idResidence: string,
    idBedroom: string
  ): Promise<IRoomEmergencyDto> {
    const returnedObject = await this.http
      .get<ResultsObject>(
        NODE_RED_IP +
          "api/rooms/emergency/" +
          idTenant +
          "/" +
          idResidence +
          "/" +
          idBedroom
      )
      .toPromise();

    if (!returnedObject) {
      return { rooms: [] };
    }

    let rooms: IRoomEmergency[] = this.resultsObjectToDto(returnedObject);
    if (!rooms) {
      return { rooms: [] };
    }

    const stoppedAlarms: IRoomEmergency[] = [];

    for (const room of rooms) {
      if (!room.alarm) {
        stoppedAlarms.push(room);
      }
    }
    const finalRooms: IRoomEmergency[] = [];
    for (const room of rooms) {
      const existentRoom = stoppedAlarms.find(
        (r) =>
          r.idBedroom === room.idBedroom &&
          r.idResidence === room.idResidence &&
          r.idTenant === room.idTenant
      );

      if (!existentRoom) {
        finalRooms.push(room);
      }
    }

    return { rooms: finalRooms };
  }

  async stopEmergency(
    idTenant: string,
    idResidence: string,
    idBedroom: string
  ): Promise<void> {
    await this.http
      .post<any>(
        NODE_RED_IP +
          "api/rooms/stop-alarm/" +
          idTenant +
          "/" +
          idResidence +
          "/" +
          idBedroom,
        {}
      )
      .toPromise();
  }

  resultsObjectToDto(resultObject: ResultsObject): any[] {
    try {
      if (!resultObject) {
        return null;
      }

      const allValues: any[] = [];

      for (const result of resultObject.results) {
        if (result && result.series) {
          for (const serie of result.series) {
            const columns: string[] = serie.columns;
            for (const val of serie.values) {
              const newObj: any = {};
              for (let i = 0; i < val.length; i++) {
                newObj[columns[i]] = val[i];
              }
              allValues.push(newObj);
            }
          }
        }
      }

      return allValues;
    } catch {
      return [];
    }
  }
}
