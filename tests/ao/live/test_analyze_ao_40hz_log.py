from scripts.analyze_ao_40hz_log import analyze_rows


def test_analyzer_accepts_25ms_loop_distribution():
    rows = [
        {"loop_dt_ms": str(v), "ao_process_ms": "3.0", "g1_dispatch_ms": "2.0"}
        for v in [0.0, 24.8, 25.0, 25.1, 25.2, 25.3, 25.4, 25.5]
    ]

    result = analyze_rows(rows)

    assert result.row_count == 8
    assert result.loop_p50_ms == 25.2
    assert result.passes_40hz_gate


def test_analyzer_rejects_100ms_loop_distribution():
    rows = [
        {"loop_dt_ms": str(v), "ao_process_ms": "3.0", "g1_dispatch_ms": "2.0"}
        for v in [0.0, 100.0, 100.1, 100.2, 100.3, 100.4, 100.5, 101.0]
    ]

    result = analyze_rows(rows)

    assert not result.passes_40hz_gate
    assert "loop_dt_ms p50" in result.failure_reasons[0]
