import xml.etree.ElementTree as ET

def joinMaps(osm, xml, outputfile):
    print 'Parsing OSM data...'
    osmtree = ET.parse(osm)
    print 'Done!'
    osmroot = osmtree.getroot()
    osmdict = {}
    print 'Creating node dictionary...'
    for osmnode in osmroot.iter('node'):
        osmdict[osmnode.get('id')]= osmnode.get('lat'), osmnode.get('lon')
    print 'Done!'
    linkdict = {}
    print 'Creating link dictionary...'
    for w in osmroot.iter('way'):
        ns = w.findall("./tag[@k='name']")
        rs = w.findall("./tag[@k='highway']")
        if len(ns) == 1 and len(rs) == 1:
            linkdict[w.get('id')]= ns[0].get('v'), rs[0].get('v')
    print 'Done!'
    xmltree = ET.parse(xml)
    xmlroot = xmltree.getroot()
    print 'Looking up nodes...'
    for xmlnode in xmlroot.iter('node'):
        latEl = ET.SubElement(xmlnode,'lat')
        lonEl = ET.SubElement(xmlnode,'lon')
        nodeID = xmlnode.find('id').text            
        if nodeID.startswith('cluster'):
            nodeID = nodeID.split('_')[1]
        if osmdict.has_key(nodeID):
            #print 'Found node ' + nodeID + ' with lat/lon ' + str(osmdict[nodeID])
            latEl.text = osmdict[nodeID][0]
            lonEl.text = osmdict[nodeID][1]
        else:
            print 'Did not find node ' + nodeID
            latEl.text = 'undefined'
            lonEl.text = 'undefined'
        links = []
        for lp in xmlnode.iter('linkPair'):
            idFrom = lp.find('idfrom').text.split('#')[0].translate(None,'-')
            idTo = lp.find('idto').text.split('#')[0].translate(None,'-')
            if linkdict.has_key(idFrom):
                links.append(linkdict[idFrom][0])
            if linkdict.has_key(idTo):
                links.append(linkdict[idTo][0])
        data = set(links)
        sortedData = sorted(data, key=lambda item: (int(item.partition(' ')[0]) if item[0].isdigit() else float('inf'), item))
        nodeDesc = ""
        if len(sortedData) > 0:
            for street in sortedData[:-1]:
                nodeDesc = nodeDesc + street + " and "
            nodeDesc = nodeDesc + sortedData[-1]
        xmlnode.find('nodeDesc').text = nodeDesc
    print 'Done!'
    for xmllink in xmlroot.iter('link'):
        linkID = xmllink.find('id').text.split('#')[0].translate(None,'-')
        descEl = ET.SubElement(xmllink, 'linkDesc')
        typeEl = ET.SubElement(xmllink, 'roadType')
    
        if linkdict.has_key(linkID):    
            descEl.text = linkdict[linkID][0]
            typeEl.text = linkdict[linkID][1]
        else:
            print 'Did not find link ' + linkID
            descEl.text = "unknown street"
    print 'Done!'
    print 'Writing output file...'
    xmltree.write(outputfile)
    print 'Done!'
    
joinMaps('nottinghamshire-justnottingham.osm', 'modum_nottingham_map.old','output.xml')
