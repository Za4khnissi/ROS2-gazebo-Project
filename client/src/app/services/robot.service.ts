import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';
import { WebSocketService } from './web-socket.service';

@Injectable({
  providedIn: 'root'
})
export class RobotService {
  private apiUrl = 'http://localhost:3000';

  constructor(private http: HttpClient, private webSocketService: WebSocketService) {}

  identifyRobot(robotId: number): Observable<{ message: string }> {
    return this.http.get<{ message: string }>(`${this.apiUrl}/robot/${robotId}/identify`);
  }

  startMission(robotId: number): Observable<{ message: string }> {
    return this.http.get<{ message: string }>(`${this.apiUrl}/mission/${robotId}/start`);
  }

  stopMission(robotId: number): Observable<{ message: string }> {
    return this.http.get<{ message: string }>(`${this.apiUrl}/mission/${robotId}/stop`);
  }

  startRos(payload: { driveModes: Record<string, string> }): Observable<any> {
    return this.http.post(`${this.apiUrl}/simulation/start_ros`, payload);
  }

  getOldLogs(): Observable<any[]> {
    return this.http.get<any[]>(`${this.apiUrl}/logs/old`);
  }

  listenForLogs(): Observable<any> {
    return this.webSocketService.listen('syncUpdate'); 
  }

  getLastStatus(): Observable<any> {
    return this.http.get<any>(`${this.apiUrl}/logs/last`);
  }
}
