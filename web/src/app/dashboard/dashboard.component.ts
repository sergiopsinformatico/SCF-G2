import { Component, OnInit } from "@angular/core";

export interface IRoom {
  readonly id: string;
  light: number;
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
  constructor() {}

  rooms: IRoom[] = [];
  ngOnInit() {
    for (let i = 0; i < 20; i++) {
      this.rooms.push({
        id: "Habitación " + i,
        light: 0,
        temperature: 0,
        humidity: 0,
        airQuality: 0,
        presence: false,
      });
    }
    this.updateInfo();
  }

  async updateInfo(): Promise<void> {
    while (true) {
      await this.delay(2000);
      for (const room of this.rooms) {
        this.updateRoom(room.id);
      }
    }
  }

  private updateRoom(roomId: string): void {
    const room = this.rooms.find((r) => r.id === roomId);
    if (room) {
      // Llamar al servidor y esas cosas
      room.light = this.randomNumber(1000, 4095, 1);
      room.humidity = this.randomNumber(0, 100, 100);
      room.airQuality = this.randomNumber(1000, 4095, 1);
      room.temperature = this.randomNumber(0, 100, 100);
    }
  }

  private randomNumber(min: number, max: number, precision: number): number {
    return (
      Math.floor(
        Math.random() * (max * precision - min * precision) + min * precision
      ) /
      (1 * precision)
    );
  }
  async delay(ms: number): Promise<PromiseConstructor> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
}
