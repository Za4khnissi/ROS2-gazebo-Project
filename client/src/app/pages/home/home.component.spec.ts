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

  it('should navigate to choose-mode page when startProcess is called', () => {
    component.startProcess();
    expect(routerSpy.navigate).toHaveBeenCalledWith(['/choose-mode']);
  });
});
