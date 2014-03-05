sel = Gui.Selection.getSelectionEx()
face = sel[0].SubObjects[0]
import Drawing
open('/tmp/out.svg', 'w').write(Drawing.projectToSVG(face, App.Vector(1, 0, 0)))
