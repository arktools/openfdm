#!/usr/bin/python
filename = 'Citation'

datcom = open('Citation.out','r')
lines = datcom.readlines()
datcom.close()

data = []
for n in range(len(lines)):
    if lines[n].find('CASEID')!=-1:
        start = lines[n].find('CASEID')+7
        end = lines[n].find(':')
        name = lines[n][start:end]
    if lines[n].find('0 ALPHA')!=-1 or lines[n].find('0   ALPHA')!=-1:
        coef = lines[n].split()
        for k in range(1,len(coef)-1):
            if data==[]:
                data.append('\t\t'+coef[k+1]+'_'+name+'.tableOnFile=false,\n')
            else:
                data.append(',\n\t\t'+coef[k+1]+'_'+name+'.tableOnFile=false,\n')
            data.append('\t\t'+coef[k+1]+'_'+name+'.table=\n\t\t{\n')
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
                    data.append('\n\t\t}')
                    
                    break
                else:
                    data.append(',\n')
                    
table = open('DatcomTable_'+filename+'.mo','w')
table.write('within OpenFDM.Aircraft;\n\nmodel DatcomTable_'+filename+'\n\textends DatcomTable(\n')
for line in data:
    table.write(line)
table.write(')\nend DatcomTable_'+filename+';')
table.close()

        
