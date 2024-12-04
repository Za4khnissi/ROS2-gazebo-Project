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
    })
      .overrideProvider(SyncGateway)
      .useValue({ server: mockServer })
      .compile();

    gateway = module.get<SyncGateway>(SyncGateway);
    gateway.server = mockServer as Server;
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
      const mockClient = { id: '12345' } as Socket;
      gateway.handleConnection(mockClient);
      expect(consoleSpy).toHaveBeenCalledWith('Client connected: 12345');
    });
  });

  describe('handleDisconnect', () => {
    it('should log client disconnection', () => {
      const consoleSpy = jest.spyOn(console, 'log');
      const mockClient = { id: '12345' } as Socket;
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
  });
});


