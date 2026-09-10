// The colour bands, straight from configs/system.yaml. Import this rather than
// repeating numbers in components — the whole point of the colouring is that a
// red chip, a red stretch of track and a red table cell are the same rule.

export const CFG = {
  latency_budget_ms: 250, inlier_gate: 25, min_quality_inliers: 40,
  alarm_error_m: 50, agl_min_m: 50, agl_max_m: 100, fixed_sigma_m: 8.0,
  prior_radius_m: 100, prior_max_age_s: 5.0, cold_start_max_tiles: 9,
};

export const BANDS = {
  ms:   { good: 250, warn: 500, unit: 'ms', label: 'latency',
          why: 'green under the 250 ms EKF3 delay budget; red past 500 ms' },
  e:    { good: 10, warn: 50, unit: 'm', label: 'position error',
          why: 'green inside 10 m; red past the 50 m alarm threshold' },
  inl:  { good: 40, warn: 25, unit: '', label: 'inliers', invert: true,
          why: 'red below the gate of 25; amber below the 40 needed for closed loop' },
  loss: { good: 6.78, warn: 25.5, unit: '', label: 'calibration loss',
          why: 'the NLL of a 10 m and a 50 m error at sigma 8 m' },
  alt:  { good: 100, warn: 100, unit: 'm', label: 'altitude', envelope: [50, 100],
          why: 'the 50-100 m AGL envelope' },
};

/** records.jsonl row -> the flat shape the drawing code reads. */
export function toRow(r, i) {
  return {
    s: r.time_step != null ? r.time_step : i + 1,
    t: r.t_unix != null ? r.t_unix : i * 0.25,
    la: r.actual_gps ? r.actual_gps.lat : null,
    lo: r.actual_gps ? r.actual_gps.lon : null,
    pla: r.predicted_gps ? r.predicted_gps.lat : null,
    plo: r.predicted_gps ? r.predicted_gps.lon : null,
    alt: r.actual_gps ? r.actual_gps.rel_alt_m : null,
    e: r.error_m, loss: r.loss, sig: r.sigma_m, inl: r.inliers, ms: r.latency_ms,
    ok: !!r.accepted, codes: r.codes || [],
  };
}
export const toRows = (records) => (records || []).map(toRow).filter((r) => r.la != null);
