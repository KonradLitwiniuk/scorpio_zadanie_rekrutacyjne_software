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

Prędkość silnika zależy od odległości od celu: sygnał = -wzmocnienie * różnica, obcięty do +/-100 i podbity do minimum 50, gdy wychodzi za mały. W strefie +/-5 ticków wysyłam 0.

Dzięki temu silnik jedzie pełną prędkością daleko od celu, a przy dojeździe zwalnia.

## Pomiary

Sterowałem silnikami ręcznie (`ros2 topic pub`) i czytałem enkodery (`ros2 topic echo`, 20 Hz).

| | Silnik 1 | Silnik 2 |
|---|---|---|
| prędkość przy 100 | ~55 ticków/s | ~80 ticków/s |
| droga hamowania od 100 | ~17 ticków | ~6-9 ticków |
| czas hamowania | ~10 próbek | ~5 próbek |
| prędkość przy 30 | ~5 ticków/s | - |

Przy minimum 50 silnik 1 jedzie ~14 ticków/s.

Parametry (1 tick około 0,09°):
- wzmocnienie: 1.0 dla silnika 1 pełna prędkość od 100 ticków, 1.5 dla silnika 2 od 65 ticków.
- sygnał minimalny: 50
- tolerancja: 5 ticków dla obu.
- próbki do zaliczenia celu: 10 dla silnika 1, 6 dla silnika 2

## Historia

Pierwsza wersja jechała stałym sygnałem +/-100 i wysyłała 0 po wjechaniu w strefę tolerancji. Ponieważ silnik po wysłaniu 0 jeszcze się toczy, tolerancję dobrałem blisko zmierzonej drogi hamowania (15 ticków dla silnika 1 i 7 dla silnika 2) - silnik zaczynał hamować tuż przed celem i zatrzymywał się prawie na nim. W trybie kolejki cel był zaliczany dopiero po tylu próbkach w strefie, ile trwa hamowanie, żeby następny punkt nie startował, zanim silnik stanie. Przy długich ruchach z testów błąd był do 4 ticków, ale przy krótkich silnik nie zdążył się rozpędzić i stawał nawet 9 ticków przed celem.

Potem dodałem stopniowe zmniejszanie prędkości, które rozwiązało ten problem - silnik dojeżdża wolno, więc prawie się nie toczy i tolerancję mogłem zmniejszyć do 5 ticków.