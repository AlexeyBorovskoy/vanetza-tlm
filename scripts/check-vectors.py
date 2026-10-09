#!/usr/bin/env python3
"""Check reference vectors with an independent ASN.1 implementation (pycrate).

For every vector in <vector directory>/manifest.json the UPER file is decoded by pycrate,
encoded again and compared byte by byte; the decoded value is compared element by element
with the XER file written by Vanetza's asn1c code. The ASN.1 modules are those Vanetza
generates its codecs from: ETSI TS 103 301 v2.1.1, ETSI TS 102 894-2 v1.3.1 (from the Vanetza
submodule) and ISO TS 19091 with ISO 24534-3 (downloaded from their public locations).

usage: check-vectors.py <vector directory> <Vanetza asn1 directory> <cache directory>
"""

import importlib
import json
import os
import sys
import urllib.request
import xml.etree.ElementTree as ElementTree

ISO_MODULES = {
    "ISO19091.asn": "https://standards.iso.org/iso/ts/19091/ed-2/en/ISO-TS-19091-addgrp-C-2018.asn",
    "ISO24534-3.asn": "https://forge.etsi.org/rep/ITS/asn1/is_ts103301/-/raw/v2.1.1/iso-patched/"
                      "ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule-patched.asn",
}
ETSI_MODULES = ["TS102894-2v131-CDD.asn", "TS103301v211-MAPEM.asn", "TS103301v211-SPATEM.asn",
                "TS103301v211-SREM.asn", "TS103301v211-SSEM.asn"]
MESSAGES = {
    "mapem": ("MAPEM_PDU_Descriptions", "MAPEM", 5),
    "spatem": ("SPATEM_PDU_Descriptions", "SPATEM", 4),
    "srem": ("SREM_PDU_Descriptions", "SREM", 9),
    "ssem": ("SSEM_PDU_Descriptions", "SSEM", 10),
}


def asn1_sources(vanetza_asn1, cache):
    os.makedirs(cache, exist_ok=True)
    paths = []
    for name, url in ISO_MODULES.items():
        path = os.path.join(cache, name)
        if not os.path.exists(path):
            urllib.request.urlretrieve(url, path)
        paths.append(path)
    paths += [os.path.join(vanetza_asn1, name) for name in ETSI_MODULES]
    return paths


def compile_modules(sources, cache):
    module = os.path.join(cache, "its_messages.py")
    if not os.path.exists(module):
        from pycrate_asn1c.asnproc import compile_text, generate_modules, PycrateGenerator
        texts = [open(path, encoding="utf-8", errors="replace").read() for path in sources]
        compile_text(texts, filenames=sources)
        generate_modules(PycrateGenerator, module)
    sys.path.insert(0, cache)
    return importlib.import_module("its_messages")


def compare(element, value, path, errors):
    """Compare a pycrate value with the asn1c XER element encoding the same value."""
    children = list(element)
    if isinstance(value, dict):  # SEQUENCE, SET
        names = [child.tag for child in children]
        if sorted(names) != sorted(value.keys()):
            errors.append("%s: XER components %s, decoded %s" % (path, names, sorted(value.keys())))
            return
        for child in children:
            compare(child, value[child.tag], path + "." + child.tag, errors)
    elif isinstance(value, list):  # SEQUENCE OF, SET OF
        if len(children) != len(value):
            errors.append("%s: %d elements in XER, %d decoded" % (path, len(children), len(value)))
            return
        for index, (child, item) in enumerate(zip(children, value)):
            compare(child, item, "%s[%d]" % (path, index), errors)
    elif isinstance(value, tuple) and len(value) == 2 and isinstance(value[0], str):  # CHOICE
        if len(children) != 1 or children[0].tag != value[0]:
            errors.append("%s: XER alternative %s, decoded %s" % (path, [c.tag for c in children], value[0]))
            return
        compare(children[0], value[1], path + "." + value[0], errors)
    elif isinstance(value, tuple) and len(value) == 2:  # BIT STRING (value, length)
        bits = "".join((element.text or "").split())
        decoded = format(value[0], "0%db" % value[1]) if value[1] else ""
        if bits != decoded:
            errors.append("%s: XER bits %s, decoded %s" % (path, bits, decoded))
    elif isinstance(value, bool):  # BOOLEAN
        if len(children) != 1 or children[0].tag != ("true" if value else "false"):
            errors.append("%s: XER %s, decoded %s" % (path, [c.tag for c in children], value))
    elif isinstance(value, int):  # INTEGER
        if (element.text or "").strip() != str(value):
            errors.append("%s: XER %r, decoded %d" % (path, element.text, value))
    elif isinstance(value, str):  # ENUMERATED (empty element named after the value) or a string
        if children:
            if len(children) != 1 or children[0].tag != value:
                errors.append("%s: XER %s, decoded %s" % (path, [c.tag for c in children], value))
        elif (element.text or "") != value:
            errors.append("%s: XER %r, decoded %r" % (path, element.text, value))
    elif isinstance(value, bytes):  # OCTET STRING
        if "".join((element.text or "").split()).lower() != value.hex():
            errors.append("%s: XER %r, decoded %s" % (path, element.text, value.hex()))
    else:
        errors.append("%s: unsupported decoded value %r" % (path, value))


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        return 2
    directory, vanetza_asn1, cache = sys.argv[1:]
    modules = compile_modules(asn1_sources(vanetza_asn1, cache), cache)
    manifest = json.load(open(os.path.join(directory, "manifest.json"), encoding="utf-8"))

    failures = 0
    for entry in manifest:
        module_name, type_name, message_id = MESSAGES[entry["message"]]
        asn_type = getattr(getattr(modules, module_name), type_name)
        uper = open(os.path.join(directory, entry["name"] + ".uper"), "rb").read()
        xer = ElementTree.parse(os.path.join(directory, entry["name"] + ".xer")).getroot()

        errors = []
        asn_type.from_uper(uper)
        value = asn_type.get_val()
        if asn_type.to_uper() != uper:
            errors.append("encoding the decoded value gives other bytes")
        if len(uper) != entry["bytes"]:
            errors.append("%d bytes, manifest says %d" % (len(uper), entry["bytes"]))
        if value["header"]["messageID"] != message_id:
            errors.append("messageID %d, expected %d" % (value["header"]["messageID"], message_id))
        if xer.tag != type_name:
            errors.append("XER root %s, expected %s" % (xer.tag, type_name))
        compare(xer, value, type_name, errors)

        failures += 1 if errors else 0
        print("%-24s %-6s %4d bytes  %s" % (entry["name"], entry["message"], len(uper),
                                             "ok" if not errors else "FAILED"))
        for error in errors[:20]:
            print("    " + error)
    print("%d of %d vectors match the independent decoder" % (len(manifest) - failures, len(manifest)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
