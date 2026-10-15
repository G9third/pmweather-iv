# PMWeather-IV 0.12.1b

- Reduces the wind-force threshold to one quarter of its original value for ground vehicles to enter sliding grip, including on hard or unclassified terrain. Reduces wind-triggered sliding grip to one quarter of its previous level. Normal driving grip is unchanged.
- Adds separate PMIV wind-force and ground-vehicle breakaway-force settings, both defaulting to 1.0 and independent of PMAero force settings.
- Keeps wind-triggered sliding grip active until the skid settles, reducing abrupt regripping while the vehicle is moving sideways.
