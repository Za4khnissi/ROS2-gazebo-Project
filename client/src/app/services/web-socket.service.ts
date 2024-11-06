import { Injectable } from '@angular/core';
import { io, Socket } from 'socket.io-client';
import { Observable } from 'rxjs';

@Injectable({
  providedIn: 'root',
})
export class WebSocketService {
  private socket: Socket;
  private readonly websocketUrl = 'http://localhost:3000';

  constructor() {
    this.socket = io(this.websocketUrl, { transports: ['websocket'] });
  }

  listen(eventName: string): Observable<any> {
    return new Observable((subscriber) => {
      this.socket.on(eventName, (data) => {
        subscriber.next(data);
      });
    });
  }

  emit(eventName: string, data: any) {
    this.socket.emit(eventName, data);
  }

  listenToBatteryStatus(robotId: number, mode: 'simulation' | 'physical'): Observable<number> {
    this.emit('clientMessage', { robotId, mode });

    return new Observable((observer) => {
      this.listen('batteryUpdate').subscribe((data: any) => {
        if (data.batteryLevel !== undefined) {
          observer.next(data.batteryLevel);
        } else {
          console.error('Battery level not found in the message:', data);
          observer.next(0);  
        }
      });

      return () => {
        this.socket.disconnect();
        console.log('WebSocket connection closed.');
      };
    });
  }
}
