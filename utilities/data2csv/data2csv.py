import csv
import pprint as pp
import math

def isfloat(x):
    try:
        a = float(x)
    except ValueError:
        return False
    else:
        return True

def isint(x):
    try:
        a = float(x)
        b = int(a)
    except ValueError:
        return False
    else:
        return a == b

def mean(values):
	return float(sum(values)) / len(values)

def std_dev(values):
	mu = mean(values)
	res = 0
	for num in values:
		res += (num - mu)**2

	res *= 1.0 / len(values)
	return math.sqrt(res)

ts = 0

log = {}
pid_str = { 
	0x10C : "RPM",
	0x10D : "SPEED",
	0x111 : "THROTTLE",
	0x104 : "ENGINE_LOAD",
	0x105 : "COOLANT_TEMP",
	0x10F : "INTAKE_TEMP",
	0x110 : "MAF_FLOW",
	0x143 : "ABS_ENGINE_LOAD",
	0x146 : "AMBIENT_TEMP",
	0x10A : "FUEL_PRESSURE",
	0x10B : "INTAKE_PRESSURE",
	0x133 : "BAROMETRIC",
	0x10E : "TIMING_ADVANCE",
	0x12F : "FUEL_LEVEL",
	0x11F : "RUNTIME",
	0x131 : "DISTANCE",
	0xF001 : "TIME_DATE",
	0xF002 : "TIME_TIME",
	0xF003 : "TIME_PLAY_SPEED",
	0xF100 : "STAT_0_60",
	0xF101 : "STAT_0_100",
	0xF102 : "STAT_0_160",
	0xF103 : "STAT_0_400",
	0xF110 : "STAT_CUR_LAP",
	0xF111 : "STAT_LAST_LAP",
	0xF112 : "STAT_BEST_LAP",
	0xF113 : "STAT_LAP_PROGRESS",
	0xFFFE : "COMMAND",
	0xFFFF : "SYNC",
	0x000A : "GPS_LATITUDE",
	0x000B : "GPS_LONGITUDE",
	0x000C : "GPS_ALTITUDE",
	0x000D : "GPS_SPEED",
	0x000E : "GPS_HEADING",
	0x000F : "GPS_SAT_COUNT",
	0x0010 : "GPS_TIME",
	0x0011 : "GPS_DATE",
	0x0020 : "ACC",
	0x0021 : "GYRO",
	0x0022 : "COMPASS",
	0x0023 : "MEMS_TEMP",
	0x0024 : "BATTERY_VOLTAGE",
	0xFF00 : "PID_VIDEO_FRAME"
			}

with open('DAT00073.CSV', 'rbU') as logfile:
	logreader = csv.reader(logfile, delimiter=',', quotechar='|')
	for row in logreader:
		elapsed = int(row[0])
		pid = int(row[1], 16) # hex
		data = row[2:]
		ts += elapsed

		if not log.has_key(pid):
			log[pid] = []

		for x in range(0,len(data)):
			if isint(data[x]):
				data[x] = int(data[x])
			else:
				data[x] = float(data[x])

		log[pid].append([ts, data])

		# print "elapsed, ", elapsed
		# print "ts, ", ts
		# print "pid, ", pid
		# print "data, ", data

with open('output.csv', 'wb') as ocsvfile:
	output = csv.writer(ocsvfile, delimiter=',')
	output.writerow("PID, Name, samples, min_elapsed (ms), max_elapsed (ms), total_elapsed (s), std_dev elapsed, avg_elapsed (s), freq (Hz), number of values, min, max, total, avg".split(','))
	for key in log.keys():
		print format(key,'04x'), '-', pid_str[key],

		elapsed_times = []

		elapsed = log[key][1][0] - log[key][0][0]
		min_elapsed = elapsed
		max_elapsed = elapsed
		total_elapsed = (log[key][len(log[key])-1][0] - log[key][0][0]) / 1000.0
		avg_elapsed = float(total_elapsed) / len(log[key])
		freq = 1 / avg_elapsed
		n_values = len(log[key][0][1])

		min = []
		max = []
		total = []

		elapsed_times.append(elapsed)

		for v in range(0,n_values):
			min.append(0)
			max.append(0)
			total.append(0)
			if(log[key][0][1][v] < log[key][1][1][v]):
				min[v] = log[key][0][1][v]
				max[v] = log[key][1][1][v]
			else:
				min[v] = log[key][1][1][v]
				max[v] = log[key][0][1][v]

			total[v] += log[key][0][1][v] + log[key][1][1][v]

		for idx in range(2,len(log[key])):
			elapsed = log[key][idx][0] - log[key][idx-1][0]
			if(elapsed < min_elapsed):
				min_elapsed = elapsed
			if(elapsed > max_elapsed):
				max_elapsed = elapsed

			elapsed_times.append(elapsed)

			for v in range(0,n_values):
				if min[v] > log[key][idx][1][v]:
					min[v] = log[key][idx][1][v]
				if max[v] < log[key][idx][1][v]:
					max[v] = log[key][idx][1][v]
				total[v] += log[key][idx][1][v]

		avg = []
		for v in range(0,n_values):
			avg.append(float(total[v]) / len(log[key]))

		print len(log[key]), min_elapsed, max_elapsed, total_elapsed, avg_elapsed, freq
		output.writerow([format(key,'#06x'), pid_str[key], len(log[key]), 
			min_elapsed, max_elapsed, total_elapsed, std_dev(elapsed_times), avg_elapsed, freq,
			n_values, str(min), str(max), str(total), str(avg)])



