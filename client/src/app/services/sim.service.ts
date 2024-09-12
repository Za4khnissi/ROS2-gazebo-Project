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

  launchSimulation(): Observable<{ message: string }> {
    return this.http.get<{ message: string }>(`${this.apiUrl}/simulation/launch`);
  }

  stopSimulation(): Observable<{ message: string }> {
    return this.http.get<{ message: string }>(`${this.apiUrl}/simulation/stop`);
  }
}
