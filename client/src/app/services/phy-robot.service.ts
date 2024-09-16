import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class RobotService {
  private apiUrl = 'http://localhost:3000';

  constructor(private http: HttpClient) {}

  identifyRobot(robotId: number): Observable<{ message: string }> {
    return this.http.post<{ message: string }>(`${this.apiUrl}/identify/physical/${robotId}`, {});
  }

  startMission(robotId: number): Observable<{ message: string }> {
    return this.http.post<{ message: string }>(`${this.apiUrl}/start-mission/physical/${robotId}`, {});
  }

  stopMission(robotId: number): Observable<{ message: string }> {
    return this.http.post<{ message: string }>(`${this.apiUrl}/stop-mission/physical/${robotId}`, {});
  }
}
