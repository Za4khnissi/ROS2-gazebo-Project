import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable, Subject } from 'rxjs';
import {ApiMissionResponse } from '@app/models/mission.model';
import { WebSocketService } from './web-socket.service';
import { MissionModel } from '@app/models/mission.model';
import { SortingInterface } from '@app/pages/history/sortingInterface';

@Injectable({
  providedIn: 'root'
})
export class RobotService {
  private apiUrl = 'http://localhost:3000';
  private apiDataUrl = 'http://localhost:3000/mission';
  private logSubject = new Subject<any>();

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

  returnFromMission(robotId: number): Observable<{ message: string }> {
    return this.http.get<{ message: string }>(`${this.apiUrl}/mission/${robotId}/return`);
  }

  changeDriveMode(robotId: number, driveMode: string): Observable<any> {
    return this.http.post(`${this.apiUrl}/robot/${robotId}/change_drive_mode`, { drive_mode: driveMode });
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

  getAllMissions(sorting: SortingInterface): Observable<ApiMissionResponse> {
    return this.http.get<ApiMissionResponse>(`${this.apiDataUrl}?_sort=${sorting.column}&_order=${sorting.order}`);
  }


  getMissionById(id: string): Observable<MissionModel> {
    return this.http.get<MissionModel>(`${this.apiUrl}/missions/${id}`);
  }
}
