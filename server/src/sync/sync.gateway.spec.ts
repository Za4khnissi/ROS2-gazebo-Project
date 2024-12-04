import { Test, TestingModule } from '@nestjs/testing';
import { SyncGateway } from './sync.gateway';
import { Server, Socket } from 'socket.io';

describe('SyncGateway', () => {
  let gateway: SyncGateway;
  let mockServer: Partial<Server>;

  beforeEach(async () => {
    mockServer = {
      emit: jest.fn(),
    };

    const module: TestingModule = await Test.createTestingModule({
      providers: [SyncGateway],
    }).compile();

    gateway = module.get<SyncGateway>(SyncGateway);
    gateway.server = mockServer as Server; // Assign the mock server
  });

  describe('afterInit', () => {
    it('should log when initialized', () => {
      const consoleSpy = jest.spyOn(console, 'log');
      gateway.afterInit();
      expect(consoleSpy).toHaveBeenCalledWith('WebSocket server initialized');
    });
  });

  describe('handleConnection', () => {
    it('should log client connection', () => {
      const consoleSpy = jest.spyOn(console, 'log');
      const mockClient = { id: '12345' } as Socket; // Mock client
      gateway.handleConnection(mockClient);
      expect(consoleSpy).toHaveBeenCalledWith('Client connected: 12345');
    });
  });

  describe('handleDisconnect', () => {
    it('should log client disconnection', () => {
      const consoleSpy = jest.spyOn(console, 'log');
      const mockClient = { id: '12345' } as Socket; // Mock client
      gateway.handleDisconnect(mockClient);
      expect(consoleSpy).toHaveBeenCalledWith('Client disconnected: 12345');
    });
  });

  describe('broadcast', () => {
    it('should emit an event with the correct payload', () => {
      const event = 'testEvent';
      const payload = { data: 'testData' };

      gateway.broadcast(event, payload);

      expect(mockServer.emit).toHaveBeenCalledWith(event, payload);
    });

    it('should log the event and payload for standard events', () => {
      const consoleSpy = jest.spyOn(console, 'log');
      const event = 'testEvent';
      const payload = { data: 'testData' };

      gateway.broadcast(event, payload);

      expect(consoleSpy).toHaveBeenCalledWith(
        `Broadcasting event: ${event}`,
        payload
      );
    });

    it('should log simplified payload for map_update events', () => {
      const consoleSpy = jest.spyOn(console, 'log');
      const event = 'map_update';
      const payload = { complex: 'data' };

      gateway.broadcast(event, payload);

      expect(consoleSpy).toHaveBeenCalledWith(`Broadcasting event: ${event}`, '...');
    });
  });

  describe('broadcastBatteryUpdate', () => {
    it('should emit a battery update with the correct data', () => {
      const robotId = 'robot123';
      const batteryLevel = 85;

      gateway.broadcastBatteryUpdate(robotId, batteryLevel);

      expect(mockServer.emit).toHaveBeenCalledWith('batteryUpdate', {
        robotId,
        batteryLevel,
      });
    });

    it('should log the battery level broadcast', () => {
      const consoleSpy = jest.spyOn(console, 'log');
      const robotId = 'robot123';
      const batteryLevel = 85;

      gateway.broadcastBatteryUpdate(robotId, batteryLevel);

      expect(consoleSpy).toHaveBeenCalledWith(
        `Battery level broadcasted for robot ${robotId}: ${batteryLevel}%`
      );
    });
  });
});

