from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add dhtxx src files.
if GetDepend('PKG_USING_MS5805'):
    src += Glob('src/ms5805.c')
    src += Glob('src/sensor_meas_ms5805.c')

if GetDepend('PKG_USING_MS5805_SAMPLE'):
    src += Glob('examples/examples_ms5805.c')

# add dhtxx include path.
path  = [cwd + '/inc']

# add src and include to group.
group = DefineGroup('ms5805', src, depend = ['PKG_USING_MS5805'], CPPPATH = path)

Return('group')
