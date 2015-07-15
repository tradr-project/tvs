#!/bin/bash

RUN=$PWD/main

for m in a1 b1 e f1 c1 d1
do
    for v in 2.400 2.700 3.000 3.300 3.600
    do
        for ms in 0.1 1.0 4.0
        do
            for cfm in 0.001 0.1 #0.5 0.95
            do
                for erp in 0.2 0.8
                do
                    for mu1 in 2.0 1.5 1.0 0.5
                    do
                        for mu2_f in 0.4 0.25 0.15
                        do
                            mu2=$(echo $mu2_f*$mu1 | bc)
                            filename=m-$m-v-$v-ms-$ms-cfm-$cfm-erp-$erp-mu1-$mu1-mu2-$mu2.csv
                            if [[ -f "$filename" ]]; then
                                echo skipping $filename because it already exists
                            else
                                clear
                                echo running $RUN -test $m $v $ms $cfm $erp $mu1 $mu2 ${filename}.tmp
                                $RUN -test $m $v $ms $cfm $erp $mu1 $mu2 ${filename}.tmp
                                mv ${filename}.tmp $filename
                            fi
                        done
                    done
                done
            done
        done
    done
done
