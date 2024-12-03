import { ComponentFixture, TestBed } from '@angular/core/testing';
import { HistoryComponent } from './history.component';
import { RobotService } from '@app/services/robot.service';
import { of, throwError } from 'rxjs';
import { MissionModel, ApiMissionResponse } from '@app/models/mission.model';
import { FormsModule } from '@angular/forms';

describe('HistoryComponent', () => {
  let component: HistoryComponent;
  let fixture: ComponentFixture<HistoryComponent>;
  let robotServiceSpy: jasmine.SpyObj<RobotService>;

  beforeEach(async () => {
    const mockRobotService = jasmine.createSpyObj('RobotService', ['getAllMissions']);

    await TestBed.configureTestingModule({
      imports: [FormsModule, HistoryComponent],
      providers: [{ provide: RobotService, useValue: mockRobotService }],
    }).compileComponents();

    robotServiceSpy = TestBed.inject(RobotService) as jasmine.SpyObj<RobotService>;
    fixture = TestBed.createComponent(HistoryComponent);
    component = fixture.componentInstance;
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  describe('fetchMissions', () => {
    it('should fetch and sort missions successfully', () => {
      const mockMissions: MissionModel[] = [
        {
          _id: '1',
          dateDebut: new Date('2024-01-01'),
          dateFin: new Date('2024-01-02'),
          duration: 24,
          robots: ['robot1'],
          isPhysical: true,
          totalDistance: 100,
          createdAt: new Date(),
          updatedAt: new Date(),
          logs: [],
        },
        {
          _id: '2',
          dateDebut: new Date('2024-01-03'),
          dateFin: new Date('2024-01-04'),
          duration: 48,
          robots: ['robot2'],
          isPhysical: false,
          totalDistance: 200,
          createdAt: new Date(),
          updatedAt: new Date(),
          logs: [],
        },
      ];

      const mockResponse: ApiMissionResponse = {
        statusCode: 200,
        missions: mockMissions,
      };

      robotServiceSpy.getAllMissions.and.returnValue(of(mockResponse));

      component.fetchMissions();
      expect(robotServiceSpy.getAllMissions).toHaveBeenCalledWith(component.sorting);
      expect(component.missions.length).toBe(2);
      expect(component.missions[0]._id).toBe('1');
    });

    it('should handle error when fetching missions fails', () => {
      spyOn(console, 'error');
      robotServiceSpy.getAllMissions.and.returnValue(throwError(() => new Error('Fetch error')));

      component.fetchMissions();
      expect(console.error).toHaveBeenCalledWith('Erreur lors de la récupération des missions :', jasmine.any(Error));
      expect(component.missions.length).toBe(0);
    });
  });

  describe('openPopup', () => {
    it('should open popup and set selectedMapData correctly', () => {
      const mockMission: MissionModel = {
        _id: '1',
        dateDebut: new Date(),
        dateFin: new Date(),
        duration: 24,
        robots: ['robot1'],
        isPhysical: true,
        totalDistance: 100,
        createdAt: new Date(),
        updatedAt: new Date(),
        mapData: {
          info: { width: 10, height: 10, data: Array(100).fill(0) },
        },
        logs: [],
      };

      const canvas = document.createElement('canvas');
      canvas.id = 'mapPopupCanvas';
      document.body.appendChild(canvas);

      component.openPopup(mockMission);

      expect(component.selectedMapData).toEqual(mockMission.mapData);
      expect(component.isPopupVisible).toBeTrue();

      const drawnCanvas = document.getElementById('mapPopupCanvas') as HTMLCanvasElement;
      expect(drawnCanvas.width).toBe(10);
      expect(drawnCanvas.height).toBe(10);

      document.body.removeChild(canvas);
    });


    it('should handle cases where no map data is available', () => {
      const mockMission: MissionModel = {
        _id: '1',
        dateDebut: new Date(),
        dateFin: new Date(),
        duration: 24,
        robots: ['robot1'],
        isPhysical: true,
        totalDistance: 100,
        createdAt: new Date(),
        updatedAt: new Date(),
        logs: [],
      };

      spyOn(console, 'warn');
      component.openPopup(mockMission);

      expect(component.selectedMapData).toBeNull();
      expect(console.warn).toHaveBeenCalledWith('Aucune donnée de carte disponible pour la mission avec l\'ID : 1');
    });
  });

  describe('closePopup', () => {
    it('should close the popup and reset state', () => {
      component.isPopupVisible = true;
      component.selectedMission = {} as MissionModel;
      component.selectedMapData = {};

      component.closePopup();
      expect(component.isPopupVisible).toBeFalse();
      expect(component.selectedMission).toBeNull();
      expect(component.selectedMapData).toBeNull();
    });
  });

  describe('filteredMissions', () => {
    it('should filter missions based on search term', () => {
      component.missions = [
        { _id: '1', dateDebut: new Date('2024-01-01'), isPhysical: true, duration: 24, totalDistance: 100 } as MissionModel,
        { _id: '2', dateDebut: new Date('2024-01-02'), isPhysical: false, duration: 48, totalDistance: 200 } as MissionModel,
      ];

      component.searchTerm = '2024-01-01';
      const filtered = component.filteredMissions();
      expect(filtered.length).toBe(1);
      expect(filtered[0]._id).toBe('1');
    });
  });

  describe('sortTable', () => {
    it('should toggle sorting order for the same column', () => {
      component.sorting = { column: 'dateDebut', order: 'asc' };
      component.sortTable('dateDebut');
      expect(component.sorting.order).toBe('desc');
    });

    it('should change sorting column and reset order to ascending', () => {
      component.sorting = { column: 'duration', order: 'desc' };
      component.sortTable('dateDebut');
      expect(component.sorting.column).toBe('dateDebut');
      expect(component.sorting.order).toBe('asc');
    });
  });
});
