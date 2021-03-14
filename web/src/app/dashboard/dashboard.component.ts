import { isDefined } from "@angular/compiler/src/util";
import { Component, OnInit } from "@angular/core";
import { IRoom, RoomsService } from "app/services/rooms.service";
import { of } from "rxjs";

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
    this.audio.src = "http://www.soundjay.com/button/beep-06.wav";
    this.audio.load();

    this.updateInfo();
  }

  async updateInfo(): Promise<void> {
    while (true) {
      const downloadedRooms = await this.roomsService.getRooms("residencia");
      for (const room of downloadedRooms.rooms) {
        const existentRoom = this.rooms.find((r) => r.nodeId === room.nodeId);

        if (!existentRoom) {
          this.rooms.push(room);
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
          existentRoom.emergency = isDefined(room.emergency)
            ? room.emergency
            : existentRoom.emergency;
        }
      }
      this.rooms.sort((a, b) =>
        a.emergency === true && b.emergency === false ? -1 : 1
      );
      let playSound = false;
      for (let room of this.rooms) {
        if (room.emergency) {
          playSound = true;
        }
      }

      if (playSound) {
        this.playAudio();
      }
      await this.delay(2000);
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
      const response = await this.roomsService.disableEmergency(
        "residencia",
        roomId
      );
      if (response && response.response === "OK") {
        existentRoom.emergency = false;
        this.rooms.sort((a, b) =>
          a.emergency === true && b.emergency === false ? -1 : 1
        );
      }
    }
  }

  async delay(ms: number): Promise<PromiseConstructor> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
}
