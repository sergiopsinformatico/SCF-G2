import { isDefined } from "@angular/compiler/src/util";
import { Component, OnInit } from "@angular/core";
import {
  IRoomData,
  IRoomEmergency,
  RoomsService,
} from "app/services/rooms.service";

interface IRoom {
  readonly nodeName: string;
  readonly nodeId: string;
  light: number;
  temperature: number;
  humidity: number;
  airQuality: number;
  presence: boolean;
  emergency: boolean;
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
      const downloadedRooms = await this.roomsService.getRooms("residencia");
      for (const room of downloadedRooms.rooms) {
        const existentRoom = this.rooms.find((r) => r.nodeId === room.nodeId);

        if (!existentRoom) {
          this.rooms.push({ ...room, emergency: false });
        } else {
          existentRoom.light = isDefined(room.light)
            ? room.light
            : existentRoom.light;
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
    }
  }

  async updateEmergency(): Promise<void> {
    let playSound: boolean;
    while (true) {
      playSound = false;
      const downloadedRooms = await this.roomsService.getEmergency(
        "residencia"
      );
      for (const room of downloadedRooms.rooms) {
        const existentRoom = this.rooms.find((r) => r.nodeId === room.nodeId);

        if (existentRoom) {
          existentRoom.emergency = room.emergency;
        }

        if (existentRoom && room.emergency) {
          playSound = true;
        }
      }
      this.rooms.sort((a, b) =>
        a.emergency === true && b.emergency === false ? -1 : 1
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

  async attendEmergency(roomId: string) {
    const existentRoom = this.rooms.find((r) => r.nodeId === roomId);
    if (existentRoom) {
      existentRoom.emergency = false;
      this.rooms.sort((a, b) =>
        a.emergency === true && b.emergency === false ? -1 : 1
      );
    }
  }

  async delay(ms: number): Promise<PromiseConstructor> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
}
