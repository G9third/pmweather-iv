"""Package current public source; dependency JARs, dev implementation and assets are excluded."""
from pathlib import Path
import argparse, datetime, hashlib, json, zipfile
root = Path(__file__).resolve().parents[1]
fixed_files = ['.gitignore', 'CONTRIBUTING.md', 'COPYING', 'COPYING.LESSER', 'LICENSE', 'README.md', 'THIRD_PARTY_NOTICES.md', 'build.bat', 'build.gradle', 'docs/ARCHITECTURE.md', 'docs/AUDIT-FIXES-20261009.md', 'docs/COMPATIBILITY-COVERAGE.md', 'docs/CONTENT-PACK-AIR-WARNING-SOUNDS.md', 'docs/RELEASE-0.11.42.md', 'gradle.properties', 'gradle/wrapper/gradle-wrapper.jar', 'gradle/wrapper/gradle-wrapper.properties', 'gradlew', 'gradlew.bat', 'libs/DEPENDENCIES.json', 'libs/README.txt', 'settings.gradle', 'tools/AnimationEvidenceRegression.java', 'tools/AuthoredSurfacePoseRegression.java', 'tools/ContactLifecycleRegression.java', 'tools/CoordinateContactRegression.java', 'tools/GeometryRegression.java', 'tools/GeometryRegression.sha256', 'tools/MaterialRegression.java', 'tools/MaterialRegression.sha256', 'tools/MovingModelHullRegression.java', 'tools/NetworkStateRegression.java', 'tools/ObserverIsolationRegression.java', 'tools/RoadVehicleCompatibilityRegression.java', 'tools/TerrainGeometryRegression.java', 'tools/TerrainGeometryRegression.sha256', 'tools/WingMomentRegression.java', 'tools/package_source.py']
fixed_files.append('docs/RELEASE-0.11.53.md')
fixed_files.extend([
 'tools/CrashGeometryCacheRegression.java',
 'tools/RigidTerrainContactRegression.java',
 'tools/RoadSuspensionRegression.java',
])
parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path)
parser.add_argument('--archive-root', default='PMWeather-IV-0.12.0-rc1-public-source')
args = parser.parse_args()
paths = {root / name for name in fixed_files}
paths.update(path for path in (root / 'src/main').rglob('*') if path.is_file())
missing = [str(path.relative_to(root)) for path in paths if not path.is_file()]
if missing: raise SystemExit('Missing public source: ' + ', '.join(missing))
paths = sorted(paths)
manifest = {
 'version': '0.12.0-rc1', 'algorithm': 'SHA-256',
 'generatedUtc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
 'privateDeveloperSourcesBundled': False, 'dependencyBinariesBundled': False,
 'contentPackAssetsBundled': False,
 'files': [{'path': p.relative_to(root).as_posix(), 'bytes': p.stat().st_size,
            'sha256': hashlib.sha256(p.read_bytes()).hexdigest()} for p in paths]}
(root / 'SOURCE-MANIFEST.json').write_text(json.dumps(manifest, indent=2)+'\n')
output = args.output or root.parent / 'PMWeather-IV-0.12.0-rc1-public-source.zip'
output.parent.mkdir(parents=True, exist_ok=True)
with zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
 for p in paths + [root/'SOURCE-MANIFEST.json']:
  z.write(p, args.archive_root+'/'+p.relative_to(root).as_posix())
with zipfile.ZipFile(output) as z:
 if z.testzip() is not None: raise SystemExit('Source ZIP integrity failure')
 for entry in manifest['files']:
  if hashlib.sha256(z.read(args.archive_root+'/'+entry['path'])).hexdigest() != entry['sha256']:
   raise SystemExit('Source hash mismatch: '+entry['path'])
print(output)
print(str(output.stat().st_size)+' bytes; source hashes verified')
