std_msgs/Header header

float32[3] basella 			# base position in lat-lon-altitude (deg, deg, m)
uint32 cycle_slip_count 	# number of cycle slips detected
uint32 roverobs				# number of observations from rover (GPS, Glonass, Gallileo, Beidou, Qzs)
uint32 baseobs				# number of observations from base (GPS, Glonass, Gallileo, Beidou, Qzs)
uint32 rovereph				# number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)
uint32 baseeph				# number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)
uint32 baseantcount			# number of base station antenna position measurements

