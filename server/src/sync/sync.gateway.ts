import {
  WebSocketGateway,
  WebSocketServer,
  OnGatewayInit,
  OnGatewayConnection,
  OnGatewayDisconnect,
} from '@nestjs/websockets';
import { Server, Socket } from 'socket.io';

@WebSocketGateway({
  cors: {
    origin: 'http://localhost:4200', // Allow requests from Angular
    methods: ['GET', 'POST'],
    credentials: true,
  },
})
export class SyncGateway
  implements OnGatewayInit, OnGatewayConnection, OnGatewayDisconnect
{
  @WebSocketServer() server: Server;

  afterInit() {
    console.log('WebSocket server initialized');
  }

  handleConnection(client: Socket) {
    console.log(`Client connected: ${client.id}`);
  }

  handleDisconnect(client: Socket) {
    console.log(`Client disconnected: ${client.id}`);
  }

  broadcast(event: string, payload: any): void {
    // Simplify the payload display for map data
    const payloadToDisplay = ['map_update', 'octomap_update'].includes(event)
      ? '...'
      : payload;
    //console.log(`Broadcasting event: ${event}`, payloadToDisplay);

    this.server.emit(event, payload);
  }

  broadcastBatteryUpdate(robotId: string, batteryLevel: number): void {
    this.server.emit('batteryUpdate', { robotId, batteryLevel });
    //console.log(`Battery level broadcasted for robot ${robotId}: ${batteryLevel}%`);
  }
}
