import { TestBed } from '@angular/core/testing';
import { HttpClientTestingModule, HttpTestingController } from '@angular/common/http/testing';
import { RobotService } from './robot.service';
import { WebSocketService } from './web-socket.service';
import { SortingInterface } from '@app/pages/history/sortingInterface';
import { ApiMissionResponse, MissionModel } from '@app/models/mission.model';

describe('RobotService', () => {
  let service: RobotService;
  let httpMock: HttpTestingController;
  let webSocketServiceSpy: jasmine.SpyObj<WebSocketService>;

  const mockApiUrl = 'http://localhost:3000';
  const mockRobotId = 1;
  const mockSorting: SortingInterface = { column: 'dateDebut', order: 'desc' };
  const mockMissionsResponse: ApiMissionResponse = {
    statusCode: 200,
    missions: [
      {
        _id: '123',
        dateDebut: new Date(),
        dateFin: new Date(),
        duration: 120,
        robots: ['robot1', 'robot2'],
        isPhysical: true,
        totalDistance: 100,
        createdAt: new Date(),
        updatedAt: new Date(),
        logs: [],
      } as MissionModel,
    ],
  };

  beforeEach(() => {
    const spy = jasmine.createSpyObj('WebSocketService', ['listen', 'emit']);

    TestBed.configureTestingModule({
      imports: [HttpClientTestingModule],
      providers: [
        RobotService,
        { provide: WebSocketService, useValue: spy },
      ],
    });

    service = TestBed.inject(RobotService);
    httpMock = TestBed.inject(HttpTestingController);
    webSocketServiceSpy = TestBed.inject(WebSocketService) as jasmine.SpyObj<WebSocketService>;
  });

  afterEach(() => {
    httpMock.verify();
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('should call identifyRobot', () => {
    const mockResponse = { message: 'Robot identified' };

    service.identifyRobot(mockRobotId).subscribe((response) => {
      expect(response.message).toBe(mockResponse.message);
    });

    const req = httpMock.expectOne(`${mockApiUrl}/robot/${mockRobotId}/identify`);
    expect(req.request.method).toBe('GET');
    req.flush(mockResponse);
  });

  it('should call startMission', () => {
    const mockResponse = { message: 'Mission started' };

    service.startMission(mockRobotId).subscribe((response) => {
      expect(response.message).toBe(mockResponse.message);
    });

    const req = httpMock.expectOne(`${mockApiUrl}/mission/${mockRobotId}/start`);
    expect(req.request.method).toBe('GET');
    req.flush(mockResponse);
  });

  it('should call stopMission', () => {
    const mockResponse = { message: 'Mission stopped' };

    service.stopMission(mockRobotId).subscribe((response) => {
      expect(response.message).toBe(mockResponse.message);
    });

    const req = httpMock.expectOne(`${mockApiUrl}/mission/${mockRobotId}/stop`);
    expect(req.request.method).toBe('GET');
    req.flush(mockResponse);
  });

  it('should call returnFromMission', () => {
    const mockResponse = { message: 'Returned from mission' };

    service.returnFromMission(mockRobotId).subscribe((response) => {
      expect(response.message).toBe(mockResponse.message);
    });

    const req = httpMock.expectOne(`${mockApiUrl}/mission/${mockRobotId}/return`);
    expect(req.request.method).toBe('GET');
    req.flush(mockResponse);
  });

  it('should call changeDriveMode', () => {
    const mockResponse = { success: true };

    service.changeDriveMode(mockRobotId, 'AUTO').subscribe((response) => {
      expect(response.success).toBe(mockResponse.success);
    });

    const req = httpMock.expectOne(`${mockApiUrl}/robot/${mockRobotId}/change_drive_mode`);
    expect(req.request.method).toBe('POST');
    expect(req.request.body).toEqual({ drive_mode: 'AUTO' });
    req.flush(mockResponse);
  });

  it('should call startRos', () => {
    const mockPayload = { driveModes: { robot1: 'AUTO', robot2: 'MANUAL' } };
    const mockResponse = { message: 'ROS started' };

    service.startRos(mockPayload).subscribe((response) => {
      expect(response.message).toBe(mockResponse.message);
    });

    const req = httpMock.expectOne(`${mockApiUrl}/simulation/start_ros`);
    expect(req.request.method).toBe('POST');
    expect(req.request.body).toEqual(mockPayload);
    req.flush(mockResponse);
  });

  it('should call getOldLogs', () => {
    const mockLogs = [{ message: 'Log 1' }, { message: 'Log 2' }];

    service.getOldLogs().subscribe((logs) => {
      expect(logs.length).toBe(2);
      expect(logs).toEqual(mockLogs);
    });

    const req = httpMock.expectOne(`${mockApiUrl}/logs/old`);
    expect(req.request.method).toBe('GET');
    req.flush(mockLogs);
  });

  it('should call getLastStatus', () => {
    const mockStatus = { status: 'OK' };

    service.getLastStatus().subscribe((status) => {
      expect(status).toEqual(mockStatus);
    });

    const req = httpMock.expectOne(`${mockApiUrl}/logs/last`);
    expect(req.request.method).toBe('GET');
    req.flush(mockStatus);
  });

  it('should call getAllMissions', () => {
    service.getAllMissions(mockSorting).subscribe((response) => {
      expect(response).toEqual(mockMissionsResponse);
    });

    const req = httpMock.expectOne(
      `${mockApiUrl}/mission?_sort=${mockSorting.column}&_order=${mockSorting.order}`
    );
    expect(req.request.method).toBe('GET');
    req.flush(mockMissionsResponse);
  });

  it('should call getMissionById', () => {
    const mockMissionId = '123';
    const mockMission = mockMissionsResponse.missions[0];

    service.getMissionById(mockMissionId).subscribe((mission) => {
      expect(mission).toEqual(mockMission);
    });

    const req = httpMock.expectOne(`${mockApiUrl}/missions/${mockMissionId}`);
    expect(req.request.method).toBe('GET');
    req.flush(mockMission);
  });
});
