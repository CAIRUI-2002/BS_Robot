package com.example.demo1.service;

import com.example.demo1.model.CablePoint;
import com.example.demo1.model.FaultPoint;
import com.example.demo1.model.InspectionResultDetail;
import com.example.demo1.model.InspectionResultRequest;
import com.example.demo1.model.InspectionResultSummary;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.jdbc.core.JdbcTemplate;
import org.springframework.stereotype.Service;

import java.sql.Timestamp;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Service
public class InspectionResultService {

    private final JdbcTemplate jdbcTemplate;
private final ObjectMapper objectMapper = new ObjectMapper();

public InspectionResultService(JdbcTemplate jdbcTemplate) {
    this.jdbcTemplate = jdbcTemplate;
}

    public Long saveInspectionResult(InspectionResultRequest req) throws Exception {
        String cablePathJson = objectMapper.writeValueAsString(req.getCablePath());
        String faultsJson = objectMapper.writeValueAsString(req.getFaults());

        Timestamp savedAt;
        if (req.getSavedAt() != null && !req.getSavedAt().isBlank()) {
            savedAt = Timestamp.from(Instant.parse(req.getSavedAt()));
        } else {
            savedAt = new Timestamp(System.currentTimeMillis());
        }

        jdbcTemplate.update(
                "INSERT INTO inspection_result(name, saved_at, cable_path_json, faults_json) VALUES (?, ?, ?, ?)",
                req.getName(),
                savedAt,
                cablePathJson,
                faultsJson
        );

        Long id = jdbcTemplate.queryForObject("SELECT LAST_INSERT_ID()", Long.class);

        // 只保留最近 5 次巡检结果
        jdbcTemplate.update("""
            DELETE FROM inspection_result
            WHERE id NOT IN (
                SELECT id FROM (
                    SELECT id FROM inspection_result
                    ORDER BY saved_at DESC, id DESC
                    LIMIT 5
                ) t
            )
        """);

        return id;
    }

    public List<InspectionResultSummary> listRecent(int limit) {
        int safeLimit = Math.min(Math.max(limit, 1), 5);

        String sql = """
            SELECT id, name, saved_at, cable_path_json, faults_json
            FROM inspection_result
            ORDER BY saved_at DESC, id DESC
            LIMIT ?
        """;

        return jdbcTemplate.query(sql, ps -> ps.setInt(1, safeLimit), rs -> {
            List<InspectionResultSummary> list = new ArrayList<>();

            while (rs.next()) {
                InspectionResultSummary item = new InspectionResultSummary();

                item.setId(rs.getLong("id"));
                item.setName(rs.getString("name"));

                Timestamp ts = rs.getTimestamp("saved_at");
                item.setSavedAt(ts == null ? null : ts.toInstant().toString());

                String cableJson = rs.getString("cable_path_json");
                String faultsJson = rs.getString("faults_json");

                List<CablePoint> cablePath = parseCablePath(cableJson);
                List<FaultPoint> faults = parseFaults(faultsJson);

                item.setPathPointCount(cablePath.size());
                item.setFaultCount(faults.size());

                list.add(item);
            }

            return list;
        });
    }

    public InspectionResultDetail getDetail(Long id) {
        String sql = """
            SELECT id, name, saved_at, cable_path_json, faults_json
            FROM inspection_result
            WHERE id = ?
        """;

        List<InspectionResultDetail> list = jdbcTemplate.query(sql, ps -> ps.setLong(1, id), rs -> {
            List<InspectionResultDetail> result = new ArrayList<>();

            while (rs.next()) {
                InspectionResultDetail detail = new InspectionResultDetail();

                detail.setId(rs.getLong("id"));
                detail.setName(rs.getString("name"));

                Timestamp ts = rs.getTimestamp("saved_at");
                detail.setSavedAt(ts == null ? null : ts.toInstant().toString());

                detail.setCablePath(parseCablePath(rs.getString("cable_path_json")));
                detail.setFaults(parseFaults(rs.getString("faults_json")));

                result.add(detail);
            }

            return result;
        });

        if (list.isEmpty()) {
            throw new RuntimeException("未找到巡检结果：" + id);
        }

        return list.get(0);
    }

    private List<CablePoint> parseCablePath(String json) {
        try {
            if (json == null || json.isBlank()) return new ArrayList<>();
            return objectMapper.readValue(json, new TypeReference<List<CablePoint>>() {});
        } catch (Exception e) {
            return new ArrayList<>();
        }
    }

    private List<FaultPoint> parseFaults(String json) {
        try {
            if (json == null || json.isBlank()) return new ArrayList<>();
            return objectMapper.readValue(json, new TypeReference<List<FaultPoint>>() {});
        } catch (Exception e) {
            return new ArrayList<>();
        }
    }
}