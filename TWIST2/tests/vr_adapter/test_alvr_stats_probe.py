from twist2_vr.alvr_stats_probe import summarize


def test_empty_capture_is_not_healthy_evidence():
    result = summarize([])
    assert not result["fresh_graph_statistics"]
    assert result["fields"] == {}
    assert result["max_reported_packets_lost_per_sec"] is None


def test_summary_keeps_graphs_and_counts_desync():
    result = summarize([
        {"event_type": {"id": "GraphStatistics", "data": {
            "total_pipeline_latency_s": .05, "server_fps": 72}}},
        {"event_type": {"id": "GraphStatistics", "data": {
            "total_pipeline_latency_s": .07, "server_fps": 70}}},
        {"event_type": {"id": "Log", "data": {
            "content": "Desync detected. Attempting recovery."}}},
        {"event_type": {"id": "StatisticsSummary", "data": {
            "packets_lost_per_sec": 0}}},
    ])
    assert result["fresh_graph_statistics"]
    assert result["desync_count"] == 1
    assert result["fields"]["server_fps"]["median"] == 71
    assert result["fields"]["total_pipeline_latency_s"]["max"] == .07
    assert result["max_reported_packets_lost_per_sec"] == 0
