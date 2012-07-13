filename = 'OpenFDM2'

datcom = open('Citation.out','r')
lines = datcom.readlines()
datcom.close()

data = ['\tmodel DatcomTable\n\t\timport Modelica.Blocks.Tables.*;\n\t\tAircraftState state;\n\t\tAerodynamicCoefficients coef;\n']
connect = ['\tequation\n']
aerocoef = ['\trecord AerodynamicCoefficients\n']
for n in range(len(lines)):
    if lines[n].find('CASEID')!=-1:
        start = lines[n].find('CASEID')+7
        end = lines[n].find(':')
        name = lines[n][start:end]
    if lines[n].find('0 ALPHA')!=-1 or lines[n].find('0   ALPHA')!=-1:
        coef = lines[n].split()
        for k in range(1,len(coef)-1):
            data.append('\t\tCombiTable1D '+coef[k+1]+'_'+name+'(columns = {2}, table =\n\t\t{\n')
            connect.append('\t\tconnect(state.alpha,'+coef[k+1]+'_'+name+'.u[1]);\n\t\tconnect(coef.'+coef[k+1]+'_'+name+','+coef[k+1]+'_'+name+'.y[1]);\n')
            aerocoef.append('\t\tReal '+coef[k+1]+'_'+name+';\n')
            a = n+2
##            data.append('\n\t{'+lines[a-1].split()[0]+',\t\t'+lines[a-1].split()[k]+'},\n')
            pos = lines[a].find(lines[a].split()[k])
            while lines[a][0]!='0':
                if lines[a][pos:(pos+len(lines[n+2].split()[k]))]==' '*len(lines[n+2].split()[k]):
                    lines[a]=lines[a].replace(' '*(len(lines[n+2].split()[k])+4),'\t'+lines[n+2].split()[k],1)
                if lines[a].split()[k]=='NDM':
                    data.append('\t\t\t{'+lines[a].split()[0]+',\t.000}')
                else:
                    data.append('\t\t\t{'+lines[a].split()[0]+',\t'+lines[a].split()[k]+'}')
##                if k == 9 or k == 10:
##                    data.append('\t{'+lines[a].split()[0]+',\t'+lines[n+2].split()[k]+'}')
##                elif k == 11:
##                    data.append('\t{'+lines[a].split()[0]+',\t'+lines[a].split()[k-2]+'}')
##                else:
##                    data.append('\t{'+lines[a].split()[0]+',\t'+lines[a].split()[k-1]+'}')
                a = a+1
                if lines[a][0]=='0' or lines[a].split()[k]=='NA':
                    data.append('\n\t\t});\n')
                    break
                else:
                    data.append(',\n')
                    
table = open(filename+'.mo','w')
table.write('package '+filename+'\n')
for line in data:
    table.write(line)
for line in connect:
    table.write(line)
table.write('\tend DatcomTable;\n')
for line in aerocoef:
    table.write(line)
table.write('\tend AerodynamicCoefficients;\n')
table.write('\trecord AircraftState\n\t\tReal p "roll rate [deg/s]";\n\t\tReal q "pitch rate [deg/s]";\n\t\tReal r "yaw rate [deg/s]";\n\t\tReal alpha;\n\t\tReal alphaDot;\n\tend AircraftState;\n')
table.write('end '+filename+';')
table.close()

        
