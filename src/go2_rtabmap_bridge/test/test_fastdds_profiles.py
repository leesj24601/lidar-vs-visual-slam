from pathlib import Path
from xml.etree import ElementTree

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
XML_NAMESPACE = {
    'fastdds': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles',
}


@pytest.mark.parametrize(
    ('profile_name', 'expected_address'),
    (
        ('fastdds_go2.xml', '192.168.123.18'),
        ('fastdds_pc.xml', '192.168.123.222'),
    ),
)
def test_fastdds_profile_uses_only_the_go2_ethernet_network(
    profile_name,
    expected_address,
):
    profile_path = REPOSITORY_ROOT / 'config' / profile_name

    root = ElementTree.parse(profile_path).getroot()
    transport = root.find(
        'fastdds:transport_descriptors/fastdds:transport_descriptor',
        XML_NAMESPACE,
    )
    participant = root.find('fastdds:participant', XML_NAMESPACE)

    assert transport is not None
    assert transport.findtext(
        'fastdds:type',
        namespaces=XML_NAMESPACE,
    ) == 'UDPv4'
    assert transport.findtext(
        'fastdds:interfaceWhiteList/fastdds:address',
        namespaces=XML_NAMESPACE,
    ) == expected_address

    assert participant is not None
    assert participant.attrib['profile_name'] == 'participant_profile_ros2'
    assert participant.attrib['is_default_profile'] == 'true'
    assert participant.findtext(
        'fastdds:rtps/fastdds:useBuiltinTransports',
        namespaces=XML_NAMESPACE,
    ) == 'false'
    assert participant.findtext(
        'fastdds:rtps/fastdds:userTransports/fastdds:transport_id',
        namespaces=XML_NAMESPACE,
    ) == transport.findtext(
        'fastdds:transport_id',
        namespaces=XML_NAMESPACE,
    )
