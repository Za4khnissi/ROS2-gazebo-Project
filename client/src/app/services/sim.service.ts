import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class SimulationService {
  private apiUrl = 'http://localhost:3000/robot';

  constructor(private http: HttpClient) {}

  identifyRobot(robotId: number): Observable<{ message: string }> {
    return this.http.post<{ message: string }>(`${this.apiUrl}/identify/simulation/${robotId}`, {});
  }

  startMission(robotId: number): Observable<{ message: string }> {
    return this.http.post<{ message: string }>(`${this.apiUrl}/start-mission/simulation/${robotId}`, {});
  }

  stopMission(robotId: number): Observable<{ message: string }> {
    return this.http.post<{ message: string }>(`${this.apiUrl}/stop-mission/simulation/${robotId}`, {});
  }

  launchSimulation(): Observable<{ message: string }> {
    return this.http.get<{ message: string }>(`${this.apiUrl}/launch-simulation`);
  }
}
