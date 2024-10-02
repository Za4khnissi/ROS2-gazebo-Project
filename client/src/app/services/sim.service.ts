import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class SimulationService {
  private apiUrl = 'http://localhost:3000';

  constructor(private http: HttpClient) {}

  identifyRobot(robotId: number): Observable<{ message: string }> {
    return this.http.get<{ message: string }>(`${this.apiUrl}/robot/${robotId}/identify`);
  }

  startMission(robotId: number): Observable<{ message: string }> {
    return this.http.get<{ message: string }>(`${this.apiUrl}/mission/${robotId}/start`);
  }

  stopMission(robotId: number): Observable<{ message: string }> {
    return this.http.get<{ message: string }>(`${this.apiUrl}/mission/${robotId}/stop`);
  }

  changeDriveMode(robotId: number, driveMode: string): Observable<any> {
    return this.http.post(`${this.apiUrl}/robot/${robotId}/change_drive_mode`, { drive_mode: driveMode });
  }
}
