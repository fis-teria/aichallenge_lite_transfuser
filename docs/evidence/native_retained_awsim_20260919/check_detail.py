from pathlib import Path
import sys
root=Path('/home/thistle/e2e_autonomous/runs/time_native_retained_awsim_20260919')
text=(root/'check_media.py').read_text().replace("kind + '_media'", "kind + '_detail_media'").replace('(18., 21., 24., 27., 30., 33.)','(24.7, 25., 25.2, 25.4, 25.6, 25.7)')
text=text.replace('duration - .8', 'duration - .11')
exec(compile(text,str(root/'check_media.py'),'exec'))
