import { isDefined } from "@angular/compiler/src/util";
import { Component, OnInit } from "@angular/core";
import {
  IRoomData,
  IRoomEmergency,
  RoomsService,
} from "app/services/rooms.service";

interface IRoom {
  readonly time: string;
  alarm: boolean;
  readonly idBedroom: string;
  readonly idResidence: string;
  readonly idTenant: string;
  lightLevel: number;
  temperature: number;
  humidity: number;
  airQuality: number;
  presence: boolean;
}

@Component({
  selector: "app-dashboard",
  templateUrl: "./dashboard.component.html",
  styleUrls: ["./dashboard.component.css"],
})
export class DashboardComponent implements OnInit {
  constructor(private roomsService: RoomsService) {}

  private audio = new Audio();
  rooms: IRoom[] = [];

  ngOnInit() {
    this.audio.src = "https://www.soundjay.com/buttons/button-41.wav";
    this.audio.load();

    this.updateInfo();
    this.updateEmergency();
  }

  async updateInfo(): Promise<void> {
    while (true) {
      const downloadedRooms = await this.roomsService.getRooms("1", "1");
      for (const room of downloadedRooms.rooms) {
        const existentRoom = this.rooms.find(
          (r) =>
            r.idBedroom === room.idBedroom &&
            r.idResidence === room.idResidence &&
            r.idTenant === room.idTenant
        );

        if (!existentRoom) {
          this.rooms.push({ ...room, alarm: false });
        } else {
          existentRoom.lightLevel = isDefined(room.lightLevel)
            ? room.lightLevel
            : existentRoom.lightLevel;
          existentRoom.temperature = isDefined(room.temperature)
            ? room.temperature
            : existentRoom.temperature;
          existentRoom.humidity = isDefined(room.humidity)
            ? room.humidity
            : existentRoom.humidity;
          existentRoom.airQuality = isDefined(room.airQuality)
            ? room.airQuality
            : existentRoom.airQuality;
          existentRoom.presence = isDefined(room.presence)
            ? room.presence
            : existentRoom.presence;
        }
      }

      await this.delay(2000);
      for (let i = 0; i < 10; i++) {
        for (const room of this.rooms) {
          const dwlRooms = await this.roomsService.getRoom(
            room.idTenant,
            room.idResidence,
            room.idBedroom
          );
          if (dwlRooms && dwlRooms.rooms) {
            for (const dwlRoom of dwlRooms.rooms) {
              room.lightLevel = isDefined(dwlRoom.lightLevel)
                ? dwlRoom.lightLevel
                : room.lightLevel;
              room.temperature = isDefined(dwlRoom.temperature)
                ? dwlRoom.temperature
                : room.temperature;
              room.humidity = isDefined(dwlRoom.humidity)
                ? dwlRoom.humidity
                : room.humidity;
              room.airQuality = isDefined(dwlRoom.airQuality)
                ? dwlRoom.airQuality
                : room.airQuality;
              room.presence = isDefined(dwlRoom.presence)
                ? dwlRoom.presence
                : room.presence;
            }
          }
 
        }
        await this.delay(2000);
      }
    }
  }

  async updateEmergency(): Promise<void> {
    let playSound: boolean;
    while (true) {
      playSound = false;
      for (const room of this.rooms) {
        const downloadedRooms = await this.roomsService.getEmergency(
          room.idTenant,
          room.idResidence,
          room.idBedroom
        );

        for (const room of downloadedRooms.rooms) {
          const existentRoom = this.rooms.find(
            (r) =>
              r.idBedroom === room.idBedroom &&
              r.idResidence === room.idResidence &&
              r.idTenant === room.idTenant
          );

          if (existentRoom) {
            existentRoom.alarm = room.alarm;
          }

          if (existentRoom && room.alarm) {
            playSound = true;
          }
        }
      }
      this.rooms.sort((a, b) =>
        a.alarm === true && b.alarm === false ? -1 : 1
      );

      if (playSound) {
        this.playAudio();
      }
      await this.delay(1000);
    }
  }

  async playAudio(): Promise<void> {
    for (let i = 0; i < 3; i++) {
      if (this.audio) {
        this.audio.play();
        await this.delay(500);
      }
    }
  }

  async attendEmergency(room: IRoom) {
    const existentRoom = this.rooms.find(
      (r) =>
        r.idBedroom === room.idBedroom &&
        r.idResidence === room.idResidence &&
        r.idTenant === room.idTenant
    );

    if (existentRoom) {
      existentRoom.alarm = false;
      await this.roomsService.stopEmergency(
        existentRoom.idTenant,
        existentRoom.idResidence,
        existentRoom.idBedroom
      );
      this.rooms.sort((a, b) =>
        a.alarm === true && b.alarm === false ? -1 : 1
      );
    }
  }

  async delay(ms: number): Promise<PromiseConstructor> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
}
