# NewMine assets

Edgar Mine Virtual STIX sections 1–5 and staging area: OpenRobotics, author Cole Biesemeyer. Fuel metadata for all six source models was checked on 2026-10-28 and declares Creative Commons Zero v1.0 Universal (CC0): https://creativecommons.org/publicdomain/zero/1.0/ . Exact source model links and advertised versions are preserved in SOURCES.json; model.config and metadata.pbtxt keep original authorship.

NewMine.sdf is the existing local scene assembly. Its poses, scaling, caps and staging safety rails are retained; absolute local mesh URIs are replaced by model:// URIs. The selected six model directories contain the local adapted meshes/textures used by the scene. Their SHA-256 values are recorded in MANIFEST.json. Fuel tip versions may differ from these adapted files; use the committed snapshot for exact reproduction.

Shared Edgar Mine Virtual STIX texture dependency (Fuel version 13) is included under models/edgar_common/materials/textures. DAE image URIs now point to these local files; original remote texture URLs have been removed from the active geometry files. Source: https://fuel.gazebosim.org/1.0/OpenRobotics/models/Edgar%20Mine%20Virtual%20STIX .
