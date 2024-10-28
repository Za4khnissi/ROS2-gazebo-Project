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
    this.socket = io(this.websocketUrl);
  }

  // Listen for events
  listen(eventName: string): Observable<any> {
    return new Observable((subscriber) => {
      this.socket.on(eventName, (data) => {
        subscriber.next(data);
      });
    });
  }

  // Emit events
  emit(eventName: string, data: any) {
    this.socket.emit(eventName, data);
  }
}
