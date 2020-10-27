from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add ms5805 src files.
if GetDepend('PKG_USING_MS5805'):
    src += Glob('Src/ms5805.c')
    src += Glob('Src/sensor_meas_ms5805.c')

if GetDepend('PKG_USING_MS5805_SAMPLE'):
    src += Glob('examples/examples_ms5805.c')

# add ms5805 include path.
path  = [cwd + '/Inc']

# add src and include to group.
group = DefineGroup('ms5805', src, depend = ['PKG_USING_MS5805'], CPPPATH = path)

Return('group')
