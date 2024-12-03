import { ComponentFixture, TestBed } from '@angular/core/testing';
import { HomeComponent } from './home.component';
import { Router } from '@angular/router';
import { RouterTestingModule } from '@angular/router/testing';

describe('HomeComponent', () => {
  let component: HomeComponent;
  let fixture: ComponentFixture<HomeComponent>;
  let router: Router;
  let navigateSpy: jasmine.Spy;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [HomeComponent, RouterTestingModule], // Importer RouterTestingModule pour simuler le routage
    }).compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(HomeComponent);
    component = fixture.componentInstance;
    router = TestBed.inject(Router);

    // Spy sur la méthode navigate pour vérifier qu'elle est appelée correctement
    navigateSpy = spyOn(router, 'navigate');
  });

  it('should create the component', () => {
    expect(component).toBeTruthy();
  });

  it('should navigate to /choose-mode when startProcess is called', () => {
    component.startProcess(); // Appel de la méthode startProcess
    expect(navigateSpy).toHaveBeenCalledWith(['/choose-mode']); // Vérification que la méthode navigate a été appelée avec le bon paramètre
  });
});
