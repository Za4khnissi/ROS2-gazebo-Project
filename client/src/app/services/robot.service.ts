import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable, Subject } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class RobotService {
  private apiUrl = 'http://localhost:3000';
  private logSubject = new Subject<any>();

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

  getOldLogs(): Observable<any[]> {
    return this.http.get<any[]>(`${this.apiUrl}/logs/old`);
  }

  listenForLogs(): Observable<any> {
    const eventSource = new EventSource(`${this.apiUrl}/logs/stream`);

    eventSource.onmessage = (event) => {
      const log = JSON.parse(event.data);
      this.logSubject.next(log);
    };

    eventSource.onerror = (error) => {
      console.error('EventSource error:', error);
      eventSource.close();
    };

    return this.logSubject.asObservable();
  }
}
