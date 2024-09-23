import { TestBed } from '@angular/core/testing';
import { HomeComponent } from './home.component';
import { Router } from '@angular/router';

describe('HomeComponent', () => {
  let component: HomeComponent;
  let routerSpy: jasmine.SpyObj<Router>;

  beforeEach(() => {
    const spy = jasmine.createSpyObj('Router', ['navigate']);

    TestBed.configureTestingModule({
      imports: [HomeComponent],
      providers: [{ provide: Router, useValue: spy }],
    }).compileComponents();

    const fixture = TestBed.createComponent(HomeComponent);
    component = fixture.componentInstance;
    routerSpy = TestBed.inject(Router) as jasmine.SpyObj<Router>;
  });

  it('should create the HomeComponent', () => {
    expect(component).toBeTruthy();
  });

  it('should have showModal initialized to false', () => {
    expect(component.showModal).toBeFalse();
  });

  it('should navigate to physical robot page', () => {
    component.goToPhysicalRobot();
    expect(routerSpy.navigate).toHaveBeenCalledWith(['/robot/physical']);
  });

  it('should navigate to simulation page', () => {
    component.goToSimulation();
    expect(routerSpy.navigate).toHaveBeenCalledWith(['/robot/simulation']);
  });
});
