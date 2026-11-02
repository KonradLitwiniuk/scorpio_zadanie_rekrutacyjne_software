# Opis rozwiązania

## Przeliczenie celu na enkodery

Silnik 1 (obrót w poziomie):
```
enc1 = -atan2(y, x) / (2*pi) * 4096
```
Minus, bo enkoder rośnie przy obrocie z +X w stronę -Y. Jak wyjdzie ujemne, dodaję 4096.

Silnik 2 (obrót w pionie):
```
enc2 = atan2(z, sqrt(x^2 + y^2)) / (2*pi) * 4096
```

Przy liczeniu różnicy pozycja - cel sprowadzam ją do zakresu [-2048, 2048], żeby silnik jechał krótszą drogą przez 0/4095.

## Sterowanie

Silnik jedzie z sygnałem ±100, a gdy jest bliżej celu niż tolerancja, dostaje 0. Silniki mają bezwładność i po wysłaniu 0 jeszcze się toczą, więc tolerancję dobrałem tak, żeby była zbliżona do drogi hamowania - wtedy silnik zatrzymuje się prawie na celu.

W trybie kolejki cel jest zaliczony, gdy silnik jest w strefie przez tyle próbek, ile trwa hamowanie. Następny cel jest brany dopiero, gdy oba silniki zaliczą obecny.

## Pomiary

Sterowałem silnikami ręcznie (`ros2 topic pub`) i czytałem enkodery (`ros2 topic echo`, 20 Hz).

| | Silnik 1 | Silnik 2 |
|---|---|---|
| prędkość przy 100 | ~55 ticków/s | ~80 ticków/s |
| droga hamowania od 100 | ~17 ticków | ~6-9 ticków |
| czas hamowania | ~10 próbek | ~5 próbek |

Na tej podstawie:
- tolerancja: 15 ticków dla silnika 1, 7 dla silnika 2 (1 tick ≈ 0,09°)
- próbki do zaliczenia celu: 10 dla silnika 1, 6 dla silnika 2

## Testy

Przykłady z README (pojedynczy cel, preempt, kolejka) - wszystkie cele osiągnięte, w preempt porzucane poprawnie, w kolejce po kolei. Największy błąd końcowy to 4 ticki (~0,35°).

## Ograniczenia

Przy krótkich ruchach silnik nie zdąży się rozpędzić i zatrzymuje się bliżej krawędzi strefy, więc błąd może dojść do wartości tolerancji (~1,3° dla silnika 1). W teście z krótkimi ruchami (13-22 ticki) silnik 1 zatrzymywał się 7-9 ticków przed celem (~0,8°).

W kolejnym kroku spróbuję dodać stopniowe zmniejszanie prędkości w miarę zbliżania się do celu. Wtedy silnik dojeżdżałby wolno i można by zmniejszyć tolerancję niezależnie od długości ruchu.
