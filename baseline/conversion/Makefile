BRANCH=hardware

all: hps_0.h baseline.rbf

hps_0.h: ../$(BRANCH)/baseline_qsys.sopcinfo
	sopc-create-header-files ../$(BRANCH)/baseline_qsys.sopcinfo --single hps_0.h --module hps_0

baseline.rbf: ../$(BRANCH)/baseline.sof
	echo 'bitstream_compression=on' >cpf_option.txt
	quartus_cpf -c -o cpf_option.txt ../$(BRANCH)/baseline.sof baseline.rbf

clean:
	rm -f hps_0.h baseline.rbf cpf_option.txt
