import os

program_name = "./main"
options = "-test"

CFM = [1e-9,1e-8,1e-7,1e-6,1e-5,1e-4,1e-3,1e-3,1e-1,1]
ERP = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8]
mux = [2.0*2.618/4.,2.0*2.618/2.,2.0*2.618,2.0*2.618*2,2.0*2.618*4]
muy = [0.25*2.618/4.,0.25*2.618/2.,0.25*2.618,0.25*2.618*2,0.25*2.618*4]
vel_scaling = 5
mass_scaling = 10

maneuvers = ["a1","a2","b1","b2","c1","c2","d1","d2","e","f1","f2"]

cfm_i = 4;
erp_i = 4;
mux_i = 2;
muy_i = 2;

maneuvers_i = 0;

line = program_name + " " + options + " " + maneuvers[maneuvers_i] + " " + str(vel_scaling) + " " + str(mass_scaling) + " " + str(CFM[cfm_i]) + " " + str(ERP[erp_i]) + " " + str(mux[mux_i]) + " " + str(muy[muy_i])+ " " + str(maneuvers_i)

os.system(line)
#os.system("./main -test a1 1e^-5 0.4 0.2 0.2")
#for it in range(0,5):
#    line = program_name + " " + options + " " + maneuvers[maneuvers_i] + " " + str(vel_scaling) + " " + str(CFM[cfm_i]) + " " + str(ERP[erp_i]) + " " + str(mux[mux_i]) + " " + str(muy[muy_i])+ " " + str(it)
#    os.system(line)
    #print line

