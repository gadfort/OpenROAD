// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// ─── SDC Visualizer Widget ──────────────────────────────────────────────────
// Tabs: Clocks · Endpoint · Port Delays · Exceptions · Clock Groups · Limits
//       · Case Analysis. See issue/sdc-visualizer-plan.md for design notes.

// Short labels + tooltips for the four timing-window components, shared
// across the Clocks stat strip, port-delay legend, and endpoint diagram.
const TIMING_LABELS = {
    tsu:       { short: 'tsu',       tip: 'library setup time — the flip-flop\'s internal setup requirement from its liberty timing arcs' },
    th:        { short: 'th',        tip: 'library hold time — the flip-flop\'s internal hold requirement from its liberty timing arcs' },
    setup_unc: { short: 'setup unc', tip: 'setup clock uncertainty — from set_clock_uncertainty -setup (jitter, CTS skew margin)' },
    hold_unc:  { short: 'hold unc',  tip: 'hold clock uncertainty — from set_clock_uncertainty -hold (jitter, CTS skew margin)' },
};

// SVG namespace; pulled out so every createElementNS call references one
// constant rather than re-typing the URI string.
const SVG_NS = 'http://www.w3.org/2000/svg';

// Tooltip text for the colored bands on the port-delay (and endpoint)
// timing diagrams, keyed by direction → band kind. Same wording the
// legend already shows; centralising it here lets the SVG <title>
// children on each band rect reuse the strings without duplicating.
const BAND_TIPS = {
    input: {
        invalid:    'data not yet valid — before the allowed arrival window',
        'hold-unc': TIMING_LABELS.hold_unc.tip,
        uncert:     'arrival window — [dMin, dMax] from set_input_delay',
        'setup-unc':TIMING_LABELS.setup_unc.tip,
        valid:      'data definitely stable — safe for the capture edge',
    },
    output: {
        valid:      'output definitely stable — ready before the capture deadline',
        'setup-unc':TIMING_LABELS.setup_unc.tip,
        uncert:     'output window — [T−dMax, T−dMin] from set_output_delay',
        'hold-unc': TIMING_LABELS.hold_unc.tip,
        invalid:    'output may still be changing — past the allowed window',
    },
};

// CSS for truncating hierarchical paths (pin / instance / net names) so
// the ellipsis lands on the LEFT side instead of the right. For names
// like `u_top/u_core/u_dut/q_reg/D` the leaf carries the meaning, so
// `…u_dut/q_reg/D` reads better than `u_top/u_core/…`.
//
// `direction: rtl` flips which edge `text-overflow: ellipsis` clips; the
// content itself stays LTR because every character (Latin letters,
// digits, underscore, slash) is either LTR-strong or bidi-neutral, so
// the bidi algorithm renders the run in the original visual order.
//
// Pair with `title = fullText` on the same element so the user can hover
// to see the part that got clipped — _setTruncatedPath wires both.
const TRUNCATE_PATH_CSS =
    'overflow:hidden;text-overflow:ellipsis;white-space:nowrap;'
    + 'direction:rtl;text-align:left;';

// Tooltip text per Limits-tab table column. Keyed by exact header
// string; entries cover the column names that show up across more than
// one section so users new to STA terminology can decode the
// abbreviations without leaving the tab.
const LIMIT_HEADER_TIPS = {
    'Clock':        'name of the clock this constraint targets',
    'Pin':          'pin the constraint is anchored to (set_clock_latency / ' +
                    'set_clock_uncertainty -to <pin>); empty when the constraint applies to a clock',
    'Port':         'top-level port the constraint targets',
    'Setup':        'setup-side value (max corner — late arrival / late edge)',
    'Hold':         'hold-side value (min corner — early arrival / early edge)',
    'Rise max':     'rise transition, max corner (late / worst-case)',
    'Rise min':     'rise transition, min corner (early / best-case)',
    'Fall max':     'fall transition, max corner (late / worst-case)',
    'Fall min':     'fall transition, min corner (early / best-case)',
    'Late rise max':  'late corner, rise transition, max delay',
    'Late rise min':  'late corner, rise transition, min delay',
    'Late fall max':  'late corner, fall transition, max delay',
    'Late fall min':  'late corner, fall transition, min delay',
    'Early rise max': 'early corner, rise transition, max delay',
    'Early rise min': 'early corner, rise transition, min delay',
    'Early fall max': 'early corner, fall transition, max delay',
    'Early fall min': 'early corner, fall transition, min delay',
    'Slew max':     'set_max_transition — maximum allowed transition (slew) time',
    'Slew min':     'set_min_transition — minimum allowed transition time',
    'Cap limit':    'set_max_capacitance — maximum allowed load capacitance',
    'Cap max':      'set_max_capacitance — maximum allowed load capacitance',
    'Cap min':      'set_min_capacitance — minimum allowed load capacitance',
    'Fanout limit': 'set_max_fanout — maximum allowed equivalent fanout load',
    'Pin cap (rise max)': 'set_load — pin (lumped) capacitance at this port',
    'Wire cap max': 'set_load -wire — wire (net) capacitance at this port',
    'Clk rise max': 'set_max_transition -clock_path — clock-network rise transition cap (max corner)',
    'Clk rise min': 'set_max_transition -clock_path — clock-network rise transition cap (min corner)',
    'Clk fall max': 'set_max_transition -clock_path — clock-network fall transition cap (max corner)',
    'Clk fall min': 'set_max_transition -clock_path — clock-network fall transition cap (min corner)',
    'Data rise max': 'set_max_transition -data_path — data-path rise transition cap (max corner)',
    'Data rise min': 'set_max_transition -data_path — data-path rise transition cap (min corner)',
    'Data fall max': 'set_max_transition -data_path — data-path fall transition cap (max corner)',
    'Data fall min': 'set_max_transition -data_path — data-path fall transition cap (min corner)',
    'Cell':         'liberty cell name (lib_cell)',
    'Scope':        'where the limit is applied (port / current_design / clock / cell / pin)',
};

// Shared layout values used by every SVG diagram on the widget. Pulling
// these out of the per-diagram method bodies removes ~5 redeclarations
// each and makes a "tighten up the row spacing" change a one-line edit.
// Values that genuinely differ between diagrams (LABEL_W, DATA_TOP,
// DATA_BOT, LANE_H — port-delay vs. multi-lane vs. single-pin) stay
// local, so each diagram can still tune its own footprint.
const DIAGRAM_CONST = {
    LEG_Y:       10,    // legend baseline above the CLK row
    CLK_HIGH:    22,    // y of clock-high level on the waveform path
    CLK_LOW:     38,    // y of clock-low level
    CLK_MID:     30,    // y of CLK row label (vertical centre)
    BAR_OPACITY: 0.85,  // band fill alpha (lets gridlines bleed through)
    LANE_GAP:    1,     // px between stacked lane rows
};

// Single source of truth for the band-kind → CSS color mapping used by every
// diagram on the widget. The data-band attribute carries the kind string so
// tests can query semantically (`[data-band="setup-unc"]`) without depending
// on which CSS variable backs it.
const BAND_COLORS = {
    'invalid':   'var(--sdc-wf-invalid)',
    'uncert':    'var(--sdc-wf-uncert)',
    'valid':     'var(--sdc-wf-valid)',
    'setup-unc': 'var(--sdc-wf-setup-unc)',
    'hold-unc':  'var(--sdc-wf-hold-unc)',
    'lib-setup': 'var(--sdc-wf-lib-setup)',
    'lib-hold':  'var(--sdc-wf-lib-hold)',
};

export class SdcWidget {
    constructor(app) {
        this._app = app;
        this._clocks = [];        // flat clock array from backend
        this._clockMap = {};      // name → clock object
        this._clockTree = [];     // root clock-tree nodes
        this._caseAnalysis = [];  // [{pin, value}]
        this._selectedClockName = null;
        this._loaded = false;
        this._loading = false;
        this._pdLoaded = false;
        this._pdLoading = false;
        this._excLoaded = false;
        this._excLoading = false;
        this._limLoaded = false;
        this._limLoading = false;
        this._cgLoaded = false;
        this._cgLoading = false;

        this._buildDom();
        // Trigger initial load after a microtask so the WebSocket connection
        // attempt has started, then readyPromise will gate the actual requests.
        Promise.resolve().then(() => {
            this._loadModes();
            this._loadData();
        });
    }

    // ── DOM construction ────────────────────────────────────────────────────

    _buildModeBar(parent) {
        const bar = document.createElement('div');
        bar.className = 'sdc-mode-bar';
        bar.style.cssText =
            'display:none;align-items:center;gap:6px;padding:3px 8px;' +
            'border-bottom:1px solid var(--border);background:var(--bg-panel);' +
            'flex-shrink:0;font-size:12px;';

        const lbl = document.createElement('span');
        lbl.textContent = 'Mode:';
        lbl.style.cssText = 'color:var(--fg-muted);';
        lbl.title = 'analysis mode (set_analysis_type / scenario). ' +
            'Switching modes selects a different SDC scenario; every tab ' +
            'is reloaded against the new mode\'s constraints.';
        bar.appendChild(lbl);

        const select = document.createElement('select');
        select.className = 'sdc-mode-select';
        select.style.cssText =
            'padding:2px 6px;font-size:12px;font-family:monospace;' +
            'background:var(--bg-input);color:var(--fg-primary);' +
            'border:1px solid var(--border);border-radius:3px;outline:none;';
        select.disabled = true;
        select.title = 'switch the active analysis mode — every tab ' +
            'reloads against the new mode\'s SDC constraints';
        select.addEventListener('change', () => {
            this._setMode(select.value);
        });
        bar.appendChild(select);

        const status = document.createElement('span');
        status.className = 'sdc-mode-status';
        status.style.cssText = 'color:var(--fg-muted);margin-left:4px;font-size:12px;';
        bar.appendChild(status);

        this._modeBar = bar;
        this._modeSelect = select;
        this._modeStatus = status;
        parent.appendChild(bar);
    }

    async _loadModes() {
        try {
            await this._app.websocketManager.readyPromise;
            const data = await this._requestWithTimeout({ type: 'sdc_list_modes' });
            this._renderModes(data);
        } catch (e) {
            // Silent: modes are optional. Older server binaries or designs
            // without MMMC will not have a modes list; hiding the bar is
            // fine. Surface the message in the status pill if the bar is
            // already visible from a prior successful load — that's how
            // the user discovers a transient timeout (e.g. refresh while
            // a long Tcl op is still running).
            console.warn('[SDC] sdc_list_modes failed:', e && e.message);
            if (this._modeBar && this._modeBar.style.display !== 'none'
                && this._modeStatus) {
                const isTimeout = !!(e && e.isTimeout);
                this._modeStatus.textContent = isTimeout
                    ? '(modes refresh timed out — try again)'
                    : `(modes refresh failed: ${(e && e.message) || e})`;
            }
        }
    }

    _renderModes(data) {
        const modes = (data && data.modes) || [];
        const current = (data && data.current) || '';
        this._modes = modes;
        this._currentModeName = current;

        // Hide the bar entirely when there is nothing meaningful to pick.
        if (modes.length < 2) {
            this._modeBar.style.display = 'none';
            return;
        }

        this._modeBar.style.display = 'flex';
        const sel = this._modeSelect;
        sel.innerHTML = '';
        for (const name of modes) {
            const opt = document.createElement('option');
            opt.value = name;
            opt.textContent = name;
            if (name === current) opt.selected = true;
            sel.appendChild(opt);
        }
        sel.disabled = false;
        this._modeStatus.textContent = '';
    }

    async _setMode(name) {
        if (!name || name === this._currentModeName) return;
        this._modeSelect.disabled = true;
        this._modeStatus.textContent = 'switching…';
        try {
            const resp = await this._requestWithTimeout({
                type: 'sdc_set_mode', mode: name });
            if (!resp || resp.ok !== true) {
                const err = resp && resp.error ? resp.error : 'unknown error';
                this._modeStatus.textContent = `failed: ${err}`;
                // Revert dropdown selection to the still-current mode.
                this._modeSelect.value = this._currentModeName || '';
                return;
            }
            this._currentModeName = resp.current || name;
            this._modeStatus.textContent = '';
            this._invalidateAllTabs();
        } catch (e) {
            this._modeStatus.textContent = `error: ${e && e.message || e}`;
            this._modeSelect.value = this._currentModeName || '';
        } finally {
            this._modeSelect.disabled = false;
        }
    }

    // Reset every tab's loaded flag + clear endpoint result cache, then reload
    // whichever tab is currently visible so the user sees the new mode.
    _invalidateAllTabs() {
        this._loaded = false;
        this._loading = false;
        this._pdLoaded = false;
        this._pdLoading = false;
        this._excLoaded = false;
        this._excLoading = false;
        this._limLoaded = false;
        this._limLoading = false;
        this._cgLoaded = false;
        this._cgLoading = false;

        // Endpoint tab is query-driven. After a mode change every previously
        // returned endpoint set is potentially stale (case_analysis can hide
        // pins, set_mode can swap clocks), so we drop the cached list and
        // re-show the populate button so the user re-triggers the walk.
        if (this._epListArea) {
            this._epListAll   = null;
            this._epListTotal = 0;
            this._epListPattern = '';
            this._epListKind    = 'all';
            this._epListArea.innerHTML =
                '<div style="padding:24px;color:var(--fg-muted);font-style:italic;">' +
                'Mode changed — click "List endpoints" to re-populate.</div>';
            if (this._epListBtn)    this._epListBtn.style.display    = '';
            if (this._epRefreshBtn) this._epRefreshBtn.style.display = 'none';
            if (this._epKindBar)    this._epKindBar.style.display    = 'none';
        }

        // CDC tab is mode-aware but doesn't need a re-fetch — the
        // overview response carries data for *every* mode in one pass,
        // so a mode switch is just a matter of re-rendering from cache.
        // We drop drill-in state (the path list + detail views show the
        // previous mode's pins which may not exist in the new one) and
        // bounce back to the matrix.
        if (this._cdcOverview) {
            this._cdcCurrentLaunch  = null;
            this._cdcCurrentCapture = null;
            this._cdcCurrentCategoryFilter = 'all';
            this._renderCdcMatrix(this._cdcOverview);
        }

        // Reload the currently-active tab.
        const active = Object.entries(this._tabPanels).find(
            ([, panel]) => panel.style.display !== 'none');
        if (active) this._activateTab(active[0]);
    }

    _buildDom() {
        const el = document.createElement('div');
        el.className = 'sdc-widget';
        el.style.cssText = 'display:flex;flex-direction:column;height:100%;overflow:hidden;font-size:13px;';

        // Shared mode bar (selector applies to all tabs)
        this._buildModeBar(el);

        // Internal tab bar
        const tabBar = document.createElement('div');
        tabBar.className = 'sdc-tab-bar';
        tabBar.style.cssText =
            'display:flex;gap:0;border-bottom:1px solid var(--border);' +
            'background:var(--bg-header);flex-shrink:0;';

        const tabNames = ['Clocks', 'Endpoint', 'Port Delays', 'Exceptions',
                          'Clock Groups', 'CDC', 'Limits', 'Case Analysis'];
        this._tabButtons = {};
        this._tabPanels = {};

        tabNames.forEach((name, i) => {
            const btn = document.createElement('button');
            btn.textContent = name;
            btn.style.cssText =
                'padding:6px 12px;border:none;cursor:pointer;font-size:12px;' +
                'background:transparent;color:var(--fg-secondary);border-bottom:2px solid transparent;';
            btn.addEventListener('click', () => this._activateTab(name));
            tabBar.appendChild(btn);
            this._tabButtons[name] = btn;

            const panel = document.createElement('div');
            panel.style.cssText = 'display:none;flex:1;overflow:hidden;';
            this._tabPanels[name] = panel;
        });

        el.appendChild(tabBar);

        const body = document.createElement('div');
        body.style.cssText = 'flex:1;overflow:hidden;display:flex;flex-direction:column;';
        tabNames.forEach(name => body.appendChild(this._tabPanels[name]));
        el.appendChild(body);

        this._buildClocksPanel(this._tabPanels['Clocks']);
        this._buildPortDelaysPanel(this._tabPanels['Port Delays']);
        this._buildExceptionsPanel(this._tabPanels['Exceptions']);
        this._buildLimitsPanel(this._tabPanels['Limits']);
        this._buildClockGroupsPanel(this._tabPanels['Clock Groups']);
        this._buildEndpointPanel(this._tabPanels['Endpoint']);
        this._buildCaseAnalysisPanel(this._tabPanels['Case Analysis']);
        this._buildCdcPanel(this._tabPanels['CDC']);

        this.element = el;
        this._activateTab('Clocks');
    }

    _activateTab(name) {
        Object.entries(this._tabButtons).forEach(([n, btn]) => {
            const active = n === name;
            btn.style.color = active ? 'var(--accent-tab)' : 'var(--fg-secondary)';
            btn.style.borderBottom = active ? '2px solid var(--accent-tab)' : '2px solid transparent';
        });
        Object.entries(this._tabPanels).forEach(([n, panel]) => {
            panel.style.display = n === name ? 'flex' : 'none';
        });
        if (name === 'Clocks' && !this._loaded && !this._loading) {
            this._loadData();
        }
        // Case-analysis data rides along on the sdc_clock_modes request that's
        // fetched as part of _loadData.  If the user opens Case Analysis before
        // they've visited Clocks, trigger that same load so the panel fills in.
        if (name === 'Case Analysis' && !this._loaded && !this._loading) {
            this._loadData();
        }
        if (name === 'Port Delays' && !this._pdLoaded && !this._pdLoading) {
            this._loadPortDelays();
        }
        if (name === 'Exceptions' && !this._excLoaded && !this._excLoading) {
            this._loadExceptions();
        }
        if (name === 'Limits' && !this._limLoaded && !this._limLoading) {
            this._loadLimits();
        }
        if (name === 'Clock Groups' && !this._cgLoaded && !this._cgLoading) {
            this._loadClockGroups();
        }
        // CDC tab is opt-in: scanning every endpoint to build the matrix is
        // the heaviest operation in this widget, so we don't auto-load. The
        // user clicks "▶ Scan CDC" once they want it.
    }

    // ── Clocks panel ────────────────────────────────────────────────────────

    _buildClocksPanel(container) {
        container.style.cssText = 'display:flex;flex-direction:column;height:100%;overflow:hidden;';

        // Toolbar row
        const toolbar = document.createElement('div');
        toolbar.style.cssText =
            'display:flex;align-items:center;padding:2px 6px;gap:6px;border-bottom:1px solid var(--border);' +
            'background:var(--bg-header);flex-shrink:0;flex-wrap:wrap;';
        const refreshBtn = document.createElement('button');
        refreshBtn.textContent = '↺ Refresh';
        refreshBtn.title = 'Reload SDC clock data from the current design';
        refreshBtn.style.cssText =
            'padding:2px 8px;font-size:12px;cursor:pointer;background:var(--bg-input);' +
            'color:var(--fg-primary);border:1px solid var(--border);border-radius:3px;';
        refreshBtn.addEventListener('click', () => {
            this._loaded = false;
            this._loading = false;
            // Re-fetch modes too so newly-defined modes (post-Tcl
            // define_scene / set_mode) show up in the dropdown
            // without forcing a widget rebuild.
            this._loadModes();
            this._loadData();
        });
        toolbar.appendChild(refreshBtn);

        // Resolve generated clocks: equivalent to running
        // `update_generated_clks` in Tcl. Generated clocks have
        // period=0 / no waveform until STA computes them from their
        // master; this button forces that computation, then reloads
        // the panel so the cards repaint with real periods.
        // Hidden by default — _renderClockCards reveals it only when at
        // least one generated clock has an unresolved period, and hides
        // it again once everything has been resolved.
        const resolveBtn = document.createElement('button');
        resolveBtn.textContent = '⟲ Resolve generated';
        resolveBtn.title =
            'Run STA\'s update_generated_clks: compute waveforms for any ' +
            'generated clocks whose period is still unresolved (shown as “—”). ' +
            'Builds the timing graph if needed — first run may take a moment.';
        resolveBtn.style.cssText =
            'display:none;padding:2px 8px;font-size:12px;cursor:pointer;' +
            'background:var(--bg-input);color:var(--fg-primary);' +
            'border:1px solid var(--border);border-radius:3px;';
        const resolveStatus = document.createElement('span');
        resolveStatus.style.cssText =
            'display:none;font-size:12px;color:var(--fg-muted);margin-left:4px;';
        this._resolveBtn = resolveBtn;
        this._resolveStatus = resolveStatus;
        resolveBtn.addEventListener('click', async () => {
            resolveBtn.disabled = true;
            const origText = resolveBtn.textContent;
            resolveBtn.textContent = '⟲ Resolving…';
            resolveStatus.textContent = '';
            resolveStatus.style.display = '';
            try {
                await this._app.websocketManager.readyPromise;
                // update_generated_clks builds the timing graph on
                // first invocation; allow extra headroom for big
                // designs (the default 60s is tight on flat designs
                // with thousands of generated clocks).
                const resp = await this._requestWithTimeout(
                    { type: 'sdc_resolve_gen_clocks' },
                    /*timeoutMs=*/300000);
                if (resp && resp.ok) {
                    resolveStatus.textContent = resp.resolved > 0
                        ? `resolved ${resp.resolved} generated clock${resp.resolved === 1 ? '' : 's'}` +
                          (resp.remaining > 0
                              ? `, ${resp.remaining} still unresolved`
                              : '')
                        : (resp.remaining > 0
                            ? `${resp.remaining} generated clock${resp.remaining === 1 ? '' : 's'} still unresolved`
                            : 'all generated clocks were already resolved');
                    // Reload Clocks data so the cards pick up the new periods.
                    // _renderClockCards will re-evaluate visibility once the
                    // refreshed clock list is in hand and hide the button if
                    // there are no unresolved generated clocks left.
                    this._loaded = false;
                    this._loading = false;
                    this._loadData();
                } else {
                    resolveStatus.textContent =
                        `failed: ${(resp && resp.error) || 'unknown error'}`;
                }
            } catch (e) {
                resolveStatus.textContent = `error: ${(e && e.message) || e}`;
            } finally {
                resolveBtn.disabled = false;
                resolveBtn.textContent = origText;
            }
        });
        toolbar.appendChild(resolveBtn);
        toolbar.appendChild(resolveStatus);

        // Filter buttons
        this._clkFilter = 'all';
        const sep = document.createElement('span');
        sep.style.cssText = 'width:1px;height:16px;background:var(--border);margin:0 2px;';
        toolbar.appendChild(sep);
        this._clkFilterBtns = this._makeFilterButtons({
            container: toolbar,
            defs: [
                { key: 'all',       label: 'All' },
                { key: 'master',    label: 'Master' },
                { key: 'generated', label: 'Generated' },
                { key: 'virtual',   label: 'Virtual' },
            ],
            initialKey: 'all',
            fontSize: '12px',
            datasetPrefix: 'clkFilter',
            onSelect: (key) => {
                this._clkFilter = key;
                this._renderClockCards();
            },
        });
        container.appendChild(toolbar);

        // Main: scrollable clock card list
        const cardScroll = document.createElement('div');
        cardScroll.style.cssText = 'flex:1;overflow-y:auto;padding:8px;'
            + 'background:var(--bg-main);min-height:0;'
            + 'scrollbar-gutter:stable;';
        this._cardScrollArea = cardScroll;
        cardScroll.innerHTML =
            '<div style="padding:12px;color:var(--fg-muted);font-style:italic;">Loading…</div>';
        container.appendChild(cardScroll);

        // Re-draw clock waveforms when the panel's width changes (split
        // pane drag, window resize, etc). The SVG widths are computed
        // from `_cardScrollArea.clientWidth` at render time, so without
        // this they stay frozen until the user hits Refresh.
        this._installResponsiveRerender('clocks', cardScroll, () => {
            if (this._loaded) this._renderClockCards();
        });

        // Sticky legend strip across the bottom of the Clocks panel.
        // Spells out the source-pin icons used in each card's details
        // section so the user doesn't have to discover them by hovering.
        // `flex-shrink:0` keeps it pinned to the bottom of the panel
        // while the scroll area absorbs all remaining space — the
        // panel's outer container is `display:flex;flex-direction:
        // column;height:100%`, so this naturally lands as a footer.
        const legend = document.createElement('div');
        legend.style.cssText =
            'flex-shrink:0;border-top:1px solid var(--border);'
            + 'background:var(--bg-header);'
            + 'padding:4px 8px;'
            + 'font-size:11px;color:var(--fg-muted);'
            + 'display:flex;align-items:center;gap:14px;flex-wrap:wrap;';
        const legendItem = (icon, label, tooltip) => {
            const span = document.createElement('span');
            span.style.cssText =
                'display:inline-flex;align-items:baseline;gap:4px;'
                + 'font-family:monospace;';
            const i = document.createElement('span');
            i.textContent = icon;
            i.style.color = 'var(--fg-primary)';
            i.style.fontSize = '12px';
            span.appendChild(i);
            const l = document.createElement('span');
            l.textContent = label;
            span.appendChild(l);
            span.title = tooltip;
            return span;
        };
        const lead = document.createElement('span');
        lead.textContent = 'source pin icons:';
        lead.style.cssText =
            'font-style:italic;color:var(--fg-muted);';
        legend.appendChild(lead);
        legend.appendChild(legendItem('⊞', 'top-level port',
            'top-level I/O port (dbBTerm) — clock arrives from outside '
            + 'the design block'));
        legend.appendChild(legendItem('◇', 'internal pin',
            'leaf-instance pin (dbITerm) — clock anchors on a flop / '
            + 'macro / sequential cell pin inside the design'));
        legend.appendChild(legendItem('◆', 'hierarchical pin',
            'hierarchical module-boundary pin (dbModITerm) — clock '
            + 'anchors on a module port rather than a leaf cell'));
        container.appendChild(legend);

        // (Case-analysis data now lives in the dedicated "Case Analysis" tab
        //  built by _buildCaseAnalysisPanel.)
    }

    // Dedicated tab for set_case_analysis + set_logic_* constraints.
    // These used to live in a cramped bottom strip of the Clocks tab where
    // long pin lists got scrolled off-screen; now they have their own page.
    _buildCaseAnalysisPanel(container) {
        container.style.cssText = 'display:flex;flex-direction:column;height:100%;overflow:hidden;';

        const toolbar = document.createElement('div');
        toolbar.style.cssText =
            'display:flex;align-items:center;padding:2px 6px;gap:6px;border-bottom:1px solid var(--border);' +
            'background:var(--bg-header);flex-shrink:0;';
        const refreshBtn = document.createElement('button');
        refreshBtn.textContent = '↺ Refresh';
        refreshBtn.title = 'Reload case-analysis and logic-value constraints';
        refreshBtn.style.cssText =
            'padding:2px 8px;font-size:12px;cursor:pointer;background:var(--bg-input);' +
            'color:var(--fg-primary);border:1px solid var(--border);border-radius:3px;';
        refreshBtn.addEventListener('click', () => {
            this._loaded = false;
            this._loading = false;
            this._loadModes();
            this._loadData();
        });
        toolbar.appendChild(refreshBtn);
        container.appendChild(toolbar);

        const scroll = document.createElement('div');
        scroll.style.cssText =
            'flex:1;overflow-y:auto;padding:8px;background:var(--bg-main);min-height:0;';
        this._caseStrip = scroll;
        container.appendChild(scroll);
    }

    _showTreePlaceholder(msg) {
        this._cardScrollArea.innerHTML =
            `<div style="padding:12px;color:var(--fg-muted);font-style:italic;">${msg}</div>`;
    }

    // ── Port Delays panel ─────────────────────────────────────────────────────

    _buildPortDelaysPanel(container) {
        container.style.cssText = 'display:flex;flex-direction:column;height:100%;overflow:hidden;';

        const toolbar = document.createElement('div');
        toolbar.style.cssText =
            'display:flex;align-items:center;padding:2px 6px;gap:6px;border-bottom:1px solid var(--border);' +
            'background:var(--bg-header);flex-shrink:0;flex-wrap:wrap;';
        const refreshBtn = document.createElement('button');
        refreshBtn.textContent = '↺ Refresh';
        refreshBtn.title = 'Reload port delay constraints from the current design';
        refreshBtn.style.cssText =
            'padding:2px 8px;font-size:12px;cursor:pointer;background:var(--bg-input);' +
            'color:var(--fg-primary);border:1px solid var(--border);border-radius:3px;';
        refreshBtn.addEventListener('click', () => {
            this._pdLoaded = false;
            this._pdLoading = false;
            this._loadModes();
            this._loadPortDelays();
        });
        toolbar.appendChild(refreshBtn);

        // Direction filter buttons
        const sep = document.createElement('div');
        sep.style.cssText = 'width:1px;height:16px;background:var(--border);margin:0 2px;';
        toolbar.appendChild(sep);

        this._pdDirFilter = 'all';
        // The label cached as `dataset.pdLabel` lets _updatePdFilterCounts
        // append "(N)" without losing the base text.
        this._pdFilterBtns = this._makeFilterButtons({
            container: toolbar,
            defs: [
                { key: 'all',    label: 'All',
                  title: 'show every constrained port (set_input_delay, ' +
                      'set_output_delay, and exception-only references)' },
                { key: 'input',  label: 'Inputs',
                  title: 'set_input_delay — arrival time relative to the ' +
                      'capturing clock edge. Diagram shows the input ' +
                      'arrival window.' },
                { key: 'output', label: 'Outputs',
                  title: 'set_output_delay — required-stable window before ' +
                      'the downstream capture edge. Diagram shows when the ' +
                      'output must be valid.' },
                { key: 'inout',  label: 'Inouts',
                  title: 'bidirectional ports — both arrival and departure ' +
                      'are constrained; each direction renders on its own ' +
                      'card.' },
            ],
            initialKey: 'all',
            datasetPrefix: 'pd',
            onSelect: (key) => {
                this._pdDirFilter = key;
                this._applyPortDelayFilter();
            },
        });

        // Clock-domain checkbox dropdown — same widget the Endpoints
        // tab uses. Filtering is client-side here (the loaded
        // _pdAllEntries list is small enough that we don't need a
        // backend round-trip on each toggle); the dropdown's
        // populate() call drives the trigger label.
        this._pdClockFilter = this._makeClockCheckboxFilter({
            container: toolbar,
            title: 'show only port delays whose reference clock matches ' +
                'one of the selected domains. "(no clock)" covers ports ' +
                'referenced only by an SDC exception, which carry no ' +
                'clock attachment.',
            onChange: () => this._applyPortDelayFilter(),
        });

        // Glob/substring search over port names — mirrors the Endpoints
        // tab's pattern input. Enter-to-search (NOT live) so the
        // search-trigger model is consistent across every SDC/CDC tab
        // — Endpoints, Port Delays, and CDC paths all wait for an
        // explicit commit. We considered live filtering (it's cheap
        // here since the data is client-side) but mixing live and
        // Enter-only across tabs led to subtle bugs where a re-render
        // mid-typing would clobber the user's in-progress input with
        // the older committed pattern; one consistent rule sidesteps
        // that whole class of issue.
        const sep2 = document.createElement('div');
        sep2.style.cssText
            = 'width:1px;height:16px;background:var(--border);margin:0 2px;';
        toolbar.appendChild(sep2);
        const pdSearch = document.createElement('input');
        pdSearch.type = 'text';
        pdSearch.placeholder
            = 'Filter ports (e.g. mem_* or reset) — Enter to search';
        pdSearch.title
            = 'Glob/substring filter on port name. Wildcards: '
            + '`*` matches any sequence, `?` matches one character. '
            + 'A pattern with no wildcards matches as a substring '
            + '(e.g. `reset` matches `top/reset_n`). '
            + 'Press Enter or click Search to apply; clearing the '
            + 'field and pressing Enter restores all ports.';
        pdSearch.style.cssText
            = 'flex:1;min-width:120px;padding:2px 6px;font-size:12px;'
            + 'font-family:monospace;background:var(--bg-input);'
            + 'color:var(--fg-primary);border:1px solid var(--border);'
            + 'border-radius:3px;outline:none;';
        this._pdSearchInput = pdSearch;
        pdSearch.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') this._applyPortDelayFilter();
        });
        toolbar.appendChild(pdSearch);
        const pdSearchBtn = document.createElement('button');
        pdSearchBtn.textContent = 'Search';
        pdSearchBtn.style.cssText
            = 'padding:2px 10px;font-size:12px;cursor:pointer;'
            + 'background:var(--bg-input);color:var(--fg-primary);'
            + 'border:1px solid var(--border);border-radius:3px;';
        pdSearchBtn.addEventListener('click',
            () => this._applyPortDelayFilter());
        toolbar.appendChild(pdSearchBtn);

        container.appendChild(toolbar);

        const scroll = document.createElement('div');
        scroll.style.cssText = 'flex:1;overflow-y:auto;padding:8px;'
            + 'background:var(--bg-main);scrollbar-gutter:stable;';
        this._pdScrollArea = scroll;
        container.appendChild(scroll);

        // Re-draw port-delay diagrams on width change. Same motivation
        // as the Clocks tab — SVG widths are baked at render time, so
        // without this the diagrams stay frozen until refresh.
        this._installResponsiveRerender('portDelays', scroll, () => {
            if (this._pdLoaded) this._applyPortDelayFilter();
        });

        scroll.innerHTML =
            '<div style="padding:12px;color:var(--fg-muted);font-style:italic;">Loading…</div>';
    }

    // Batch size for paginated fetches (port delays + endpoint result list).
    // Centralized so it's a one-line change to retune for performance vs.
    // perceived-latency tradeoffs.
    _batchSize() { return 100; }

    // CDC path-list page size — bigger than `_batchSize()` because the
    // backend's per-page work is just an in-memory scan of the cached
    // pair vector (the heavy walk happens once on cdc_overview), so a
    // wider page costs almost nothing on the server, fills the table
    // in fewer round-trips, and reduces scroll-driven follow-up
    // requests on dense (clk_a → clk_b) cells.
    _cdcPathsBatchSize() { return 1000; }

    // Compile a user-typed pattern to a predicate `(name) => boolean`.
    // Mirrors the affordance of OpenSTA's `sta::PatternMatch` plus the
    // shorthand the SDC tab's other glob inputs already imply: a bare
    // word with no wildcards matches as a substring (so `reset`
    // matches `top/reset_n` without forcing the user to type
    // `*reset*`); patterns with `*` / `?` are treated as full globs
    // anchored to the whole name. Returns a tautology predicate when
    // the pattern is empty so callers can skip null-checks.
    _compileGlob(pattern) {
        const p = (pattern || '').trim();
        if (!p) return () => true;
        const hasWildcard = /[*?]/.test(p);
        const escaped
            = p.replace(/[.+^${}()|[\]\\]/g, '\\$&')
                .replace(/\*/g, '.*').replace(/\?/g, '.');
        const re = hasWildcard
            ? new RegExp('^' + escaped + '$')
            : new RegExp(escaped);
        return (name) => re.test(name || '');
    }

    async _loadPortDelays() {
        if (this._pdLoaded || this._pdLoading) return;
        this._pdLoading = true;
        this._pdScrollArea.innerHTML =
            '<div style="padding:12px;color:var(--fg-muted);font-style:italic;">Loading…</div>';
        try {
            await this._app.websocketManager.readyPromise;
            // First batch: offset=0, limit=BATCH. Renders immediately so the
            // user sees something. Subsequent batches stream in via the
            // scroll listener attached below.
            const data = await this._requestWithTimeout({
                type: 'sdc_port_delays',
                offset: 0,
                limit: this._batchSize(),
            });
            this._renderPortDelays(data);
            this._pdLoaded = true;
            this._installPdInfiniteScroll();
            this._maybeTopUpPdList();
        } catch (e) {
            console.error('[SDC] port delays load error:', e);
            this._showLoadError(this._pdScrollArea, 'port delays', e, () => {
                this._pdLoaded = false;
                this._pdLoading = false;
                this._loadPortDelays();
            });
        } finally {
            this._pdLoading = false;
        }
    }

    _renderPortDelays(data) {
        this._pdAllEntries = data.port_delays || [];
        this._pdTotal      = (typeof data.total === 'number')
            ? data.total : this._pdAllEntries.length;
        this._pdTimeUnit   = data.time_unit || 'ns';
        this._pdCapUnit    = data.cap_unit  || 'pF';
        // Cache full-list direction totals so paginated follow-on
        // batches don't have to re-derive (and re-grow) the counts.
        this._pdDirectionsTotal = data.directions_total || null;
        this._updatePdFilterCounts();
        if (this._pdClockFilter) {
            this._pdClockFilter.populate(data.clocks_total || {});
        }
        this._applyPortDelayFilter();
    }

    // Update the direction-filter button labels with stable totals
    // computed by the backend over the FULL port list (not the page).
    // Without this the labels grew as the user scrolled and pages
    // streamed in, making it look like ports were being added live.
    // Falls back to a client-side scan over `_pdAllEntries` if the
    // backend response didn't carry `directions_total` (older server).
    _updatePdFilterCounts() {
        if (!this._pdFilterBtns) return;
        // Counts always come from a client-side scan of `_pdAllEntries`
        // so they can apply the user's current pattern and clock
        // filters. The backend-supplied `_pdDirectionsTotal` only
        // matches when no client-side filters are active — using it
        // unconditionally produced stale chip counts that didn't
        // update on search (chips would show design-wide totals while
        // the cards below reflected the pattern match).
        const entries = this._pdAllEntries || [];
        const pattern = (this._pdSearchInput
                         && this._pdSearchInput.value) || '';
        const nameOk = this._compileGlob(pattern);
        const clkOk = this._pdClockFilter
            ? (e) => this._pdClockFilter.matches(e.clock)
            : () => true;

        // Per-chip the count answers "how many ports would match if I
        // selected this direction filter, with my current pattern and
        // clock filters?" — so each chip's count applies all OTHER
        // filters but not its own direction. The "all" chip applies
        // pattern + clock without any direction restriction.
        const seen = {
            input: new Set(), output: new Set(), inout: new Set(),
        };
        const seenAll = new Set();
        for (const e of entries) {
            if (!nameOk(e.port) || !clkOk(e)) continue;
            const dir = e.direction || (e.is_input ? 'input' : 'output');
            seenAll.add(e.port);
            if (seen[dir]) seen[dir].add(e.port);
        }
        const counts = {
            all:    seenAll.size,
            input:  seen.input.size,
            output: seen.output.size,
            inout:  seen.inout.size,
        };
        for (const [key, btn] of Object.entries(this._pdFilterBtns)) {
            const base = btn.dataset.pdLabel || btn.textContent;
            const n = counts[key] || 0;
            btn.textContent = `${base} (${n})`;
        }
    }

    // Install a width-only ResizeObserver on `scrollEl` that calls
    // `rerenderFn` (debounced via rAF) whenever the element's
    // clientWidth changes meaningfully. Used by the Clocks, Port
    // Delays, and Endpoints tabs to redraw their SVG waveforms when
    // the panel is resized — the SVGs bake their width into pixel
    // attributes at render time, so without this the diagrams stay
    // frozen until the user hits Refresh.
    //
    // Three layers of flicker protection:
    //   1) Width-only filter: a card expanding downstream of the
    //      diagram changes height, not width, so the redraw skips.
    //   2) Threshold (`MIN_DELTA`): a 4px jitter floor so sub-pixel
    //      layout shifts don't trigger redraws. Scrollbar appearance
    //      is normally handled by `scrollbar-gutter:stable` on the
    //      scroll element, but the threshold catches browsers that
    //      don't honour the property.
    //   3) Re-entrant guard (`_resizeRerenderActive`): the observer
    //      no-ops while the rerender is itself running, since the
    //      rerender clears innerHTML which can momentarily change
    //      clientWidth and would otherwise feed back as a fresh
    //      resize event.
    //
    // Keyed by `tabName` so each tab owns its own observer; switching
    // tabs leaves the others quietly observing — they get clientWidth
    // = 0 when their pane is hidden and the early-out skips.
    _installResponsiveRerender(tabName, scrollEl, rerenderFn) {
        if (!scrollEl || typeof ResizeObserver === 'undefined') return;
        this._tabResizeObservers ??= {};
        if (this._tabResizeObservers[tabName]) {
            this._tabResizeObservers[tabName].disconnect();
        }
        const MIN_DELTA = 4;
        let lastW = scrollEl.clientWidth;
        let pending = false;
        const obs = new ResizeObserver(() => {
            if (this._resizeRerenderActive) return;
            const w = scrollEl.clientWidth;
            if (w === 0) return;
            if (Math.abs(w - lastW) < MIN_DELTA) return;
            lastW = w;
            if (pending) return;
            pending = true;
            requestAnimationFrame(() => {
                pending = false;
                this._resizeRerenderActive = true;
                try {
                    rerenderFn();
                } catch (e) {
                    console.warn('[SDC] resize re-render failed for '
                        + tabName + ':', e);
                }
                // Let the layout settle before re-arming. Without
                // this delay, async work kicked off by the rerender
                // (lazy fetches in expanded endpoint cards, etc.)
                // would land after we re-armed and could feed a
                // fresh "width changed" event back through.
                setTimeout(() => {
                    this._resizeRerenderActive = false;
                    // Refresh `lastW` so the post-settle width is
                    // the new baseline — if `scrollbar-gutter` was
                    // ineffective and the bar appeared during the
                    // grace window, we don't want the next observer
                    // fire to count that as a real change.
                    lastW = scrollEl.clientWidth;
                }, 100);
            });
        });
        obs.observe(scrollEl);
        this._tabResizeObservers[tabName] = obs;
    }

    // Scroll-near-bottom triggers an incremental fetch of the next batch
    // and appends to `_pdAllEntries`. The handler is debounced via
    // `_pdFetchingMore` so repeated scroll events don't pile up.
    _installPdInfiniteScroll() {
        if (this._pdScrollHandler) return;  // attached once
        const handler = () => {
            const el = this._pdScrollArea;
            if (!el) return;
            // Already have everything, or a fetch is in flight.
            if (this._pdFetchingMore) return;
            if (this._pdAllEntries.length >= this._pdTotal) return;
            // Trigger when within a viewport-and-a-bit of the bottom.
            const slack = el.clientHeight * 1.5;
            if (el.scrollTop + el.clientHeight + slack >= el.scrollHeight) {
                this._fetchPdMore();
            }
        };
        this._pdScrollArea.addEventListener('scroll', handler);
        this._pdScrollHandler = handler;
    }

    async _fetchPdMore() {
        if (this._pdFetchingMore) return;
        if (this._pdAllEntries.length >= this._pdTotal) return;
        this._pdFetchingMore = true;
        this._showPdMoreFooter('Loading more…');
        try {
            const data = await this._requestWithTimeout({
                type: 'sdc_port_delays',
                offset: this._pdAllEntries.length,
                limit:  this._batchSize(),
            });
            const more = data.port_delays || [];
            this._pdAllEntries = this._pdAllEntries.concat(more);
            if (typeof data.total === 'number') this._pdTotal = data.total;
            // Refresh the cached direction totals from the server in
            // case anything changed mid-pagination (rare but cheap to
            // pick up — keeps the filter buttons honest).
            if (data.directions_total) {
                this._pdDirectionsTotal = data.directions_total;
            }
            this._updatePdFilterCounts();
            // Append-only — keep all existing DOM in place and add cards
            // for the new entries. No innerHTML reset means no scroll jump
            // and no flicker as the user scrolls down.
            this._appendPortDelayBatch(more);
        } catch (e) {
            console.warn('[SDC] port delays paginated fetch failed', e);
            this._showPdMoreFooter(
                `Failed to load more: ${(e && e.message) || e}`);
        } finally {
            this._pdFetchingMore = false;
            this._maybeTopUpPdList();
        }
    }

    // Same trick as _maybeTopUpEpList: when the rendered cards don't
    // fill the viewport, the scroll handler can never fire and the
    // user gets stuck on the first batch with no scrollbar to drag.
    // Top up until the area scrolls or we've drained the list.
    _maybeTopUpPdList() {
        if (typeof requestAnimationFrame === 'undefined') return;
        requestAnimationFrame(() => {
            const el = this._pdScrollArea;
            if (!el) return;
            if (this._pdFetchingMore) return;
            if (!this._pdAllEntries) return;
            if (this._pdAllEntries.length >= (this._pdTotal || 0)) return;
            if (el.scrollHeight <= el.clientHeight + 16) {
                this._fetchPdMore();
            }
        });
    }

    _showPdMoreFooter(text) {
        // Replace any existing footer; appended at end of scroll area.
        let f = this._pdScrollArea.querySelector('.sdc-pd-more-footer');
        if (!f) {
            f = document.createElement('div');
            f.className = 'sdc-pd-more-footer';
            f.style.cssText =
                'padding:10px;text-align:center;font-size:12px;' +
                'color:var(--fg-muted);font-style:italic;';
            this._pdScrollArea.appendChild(f);
        }
        f.textContent = text;
    }

    // Compute a per-clock port-count map from `_pdAllEntries` under
    // the current pattern + direction filters (but NOT the clock
    // filter — each row's count answers "switching to this clock
    // would show me N ports given my other filters"). Same convention
    // the backend's clocks_total uses for the Endpoints tab.
    _pdComputeClockCounts() {
        const entries = this._pdAllEntries || [];
        const pattern = (this._pdSearchInput
                         && this._pdSearchInput.value) || '';
        const nameOk = this._compileGlob(pattern);
        const dirFilter = this._pdDirFilter || 'all';
        const dirOk = dirFilter === 'all'
            ? () => true
            : (e) => (e.direction
                      || (e.is_input ? 'input' : 'output')) === dirFilter;
        const counts = {};
        const seen = {};
        for (const e of entries) {
            if (!nameOk(e.port) || !dirOk(e)) continue;
            // Null/undefined clock collapses under the widget's
            // `__none__` sentinel, which `_makeClockCheckboxFilter`'s
            // populate() lifts into a "(no clock)" row. Common case:
            // exception-only ports (set_max_delay / set_input_delay
            // with no `-clock`) — they have no clock field but the
            // user still wants to see + filter them.
            const k = e.clock == null ? '__none__' : e.clock;
            if (!seen[k]) seen[k] = new Set();
            if (!seen[k].has(e.port)) {
                seen[k].add(e.port);
                counts[k] = (counts[k] || 0) + 1;
            }
        }
        return counts;
    }

    _applyPortDelayFilter() {
        this._pdScrollArea.innerHTML = '';
        // Refresh the direction-chip counts so they reflect the new
        // search pattern + clock filter. Without this the chips kept
        // showing design-wide totals while the cards below already
        // reflected the user's filter — confusing on a search like
        // "u_pad*" that matches 12 of 200 input ports.
        this._updatePdFilterCounts();
        // Refresh the clock dropdown's per-clock counts and visible
        // entries the same way: clocks that no longer match the
        // pattern shouldn't sit in the dropdown with their old
        // design-wide totals (or appear at all if zero match).
        // Selection state is preserved by `populate` for any clock
        // still in the new map.
        if (this._pdClockFilter) {
            this._pdClockFilter.populate(this._pdComputeClockCounts());
        }
        const entries  = this._pdAllEntries || [];
        const timeUnit = this._pdTimeUnit   || 'ns';
        const filter   = this._pdDirFilter  || 'all';
        const pattern  = (this._pdSearchInput
                          && this._pdSearchInput.value) || '';
        const nameOk   = this._compileGlob(pattern);

        if (entries.length === 0) {
            this._pdScrollArea.innerHTML =
                '<div style="padding:12px;color:var(--fg-muted);font-style:italic;">No port delays defined in SDC.</div>';
            return;
        }

        // Apply direction + clock-domain + name filters using the
        // backend-supplied direction / clock fields. Clock filter
        // accepts null clock as the "(no clock)" sentinel —
        // exception-only entries fall there.
        const dirOk = filter === 'all'
            ? () => true
            : (e) => (e.direction || (e.is_input ? 'input' : 'output')) === filter;
        const clkOk = this._pdClockFilter
            ? (e) => this._pdClockFilter.matches(e.clock)
            : () => true;
        const filtered = entries.filter(
            e => dirOk(e) && clkOk(e) && nameOk(e.port));

        if (filtered.length === 0) {
            const msg = { input: 'inputs', output: 'outputs', inout: 'inout ports' }[filter] || filter;
            const patternHint = pattern.trim()
                ? ` matching "${pattern.trim()}"`
                : '';
            this._pdScrollArea.innerHTML =
                `<div style="padding:12px;color:var(--fg-muted);font-style:italic;">No ${msg}${patternHint} match the current filters.</div>`;
            return;
        }

        // Group by port so multiple clock entries appear under one port card.
        // For inout ports we keep the *single* card (Q4 of the design review),
        // but split entries internally into "Input delays" / "Output delays"
        // sub-sections by is_input so each sub-section can have its own
        // (mirrored) diagram orientation.
        const groups = new Map();
        for (const e of filtered) {
            if (!groups.has(e.port)) groups.set(e.port, []);
            groups.get(e.port).push(e);
        }

        for (const [portName, portEntries] of groups) {
            const card = this._buildPortDelayCard(portName, portEntries, timeUnit);
            this._pdScrollArea.appendChild(card);
        }
        this._updatePdFooter();
    }

    // Append-only path for paginated fetches: build cards for the new
    // entries and APPEND to the scroll area without clearing innerHTML.
    // Re-rendering would collapse the scroll content (resetting scrollTop
    // to 0) and re-create every SVG diagram — visible flicker on long
    // lists. By appending we keep all existing DOM in place; only the
    // newly-loaded ports cause work.
    //
    // Cross-batch edge case: if a port has entries split across two
    // batches (rare — backend sorts entries by port order), the first
    // card already exists. We replace just that one card with the merged
    // version so the user sees all entries for that port together.
    _appendPortDelayBatch(newEntries) {
        if (!newEntries || newEntries.length === 0) {
            this._updatePdFooter();
            return;
        }
        const filter   = this._pdDirFilter || 'all';
        const timeUnit = this._pdTimeUnit  || 'ns';
        const dirOk = filter === 'all'
            ? () => true
            : (e) => (e.direction || (e.is_input ? 'input' : 'output')) === filter;
        const clkOk = this._pdClockFilter
            ? (e) => this._pdClockFilter.matches(e.clock)
            : () => true;
        const filtered = newEntries.filter(e => dirOk(e) && clkOk(e));
        if (filtered.length === 0) {
            this._updatePdFooter();
            return;
        }
        // If there's currently a "no entries" placeholder, drop it before
        // we start appending real cards. (Possible when the very first
        // load returned 0 entries but a later batch has data.)
        const placeholder = this._pdScrollArea.querySelector(
            ':scope > div[style*="font-style:italic"]');
        if (placeholder && !placeholder.classList.contains('sdc-pd-more-footer')) {
            placeholder.remove();
        }
        const footer = this._pdScrollArea.querySelector('.sdc-pd-more-footer');

        // Group new entries by port (preserving first-seen order).
        const newByPort = new Map();
        for (const e of filtered) {
            if (!newByPort.has(e.port)) newByPort.set(e.port, []);
            newByPort.get(e.port).push(e);
        }

        // Pre-index existing cards by port name. Avoids CSS.escape (not
        // available in older Node test envs) and handles SDC port names
        // that may contain bracket/quote characters.
        const existingByPort = new Map();
        for (const child of this._pdScrollArea.children) {
            if (child.dataset && child.dataset.pdPort) {
                existingByPort.set(child.dataset.pdPort, child);
            }
        }
        for (const [portName] of newByPort) {
            // Use *all* entries for this port (the existing batch may have
            // contributed some). This handles the cross-batch case cleanly.
            const allForPort = this._pdAllEntries.filter(e =>
                e.port === portName &&
                (filter === 'all' ||
                 (e.direction || (e.is_input ? 'input' : 'output')) === filter));
            const newCard = this._buildPortDelayCard(portName, allForPort, timeUnit);
            const existing = existingByPort.get(portName);
            if (existing) {
                existing.replaceWith(newCard);
            } else if (footer) {
                this._pdScrollArea.insertBefore(newCard, footer);
            } else {
                this._pdScrollArea.appendChild(newCard);
            }
        }
        this._updatePdFooter();
    }

    // Build the wrapper element for one port's set of entries. Used by
    // both the full-render path (`_applyPortDelayFilter`) and the
    // append-only path (`_appendPortDelayBatch`).
    _buildPortDelayCard(portName, portEntries, timeUnit) {
        const wrapper = document.createElement('div');
        wrapper.dataset.pdPort = portName;
        wrapper.style.cssText =
            'margin-bottom:12px;border:1px solid var(--border);border-radius:4px;overflow:hidden;';

        // Port header with direction badge — pick the badge dir from the
        // entries we actually have (any entry; direction is the same
        // across all entries for a given port).
        const header = document.createElement('div');
        const e0 = portEntries[0];
        const dir = e0.direction || (e0.is_input ? 'input' : 'output');
        header.style.cssText =
            'display:flex;align-items:center;gap:8px;padding:4px 8px;' +
            'background:var(--bg-header);border-bottom:1px solid var(--border);font-size:12px;';
        const badge = document.createElement('span');
        const badgeStyle = dir === 'input'
            ? 'background:var(--sdc-input-bg);color:var(--sdc-input-fg);'
            : dir === 'output'
            ? 'background:var(--sdc-output-bg);color:var(--sdc-output-fg);'
            : 'background:var(--sdc-inout-bg);color:var(--sdc-inout-fg);';
        badge.style.cssText =
            `font-size:11px;padding:1px 5px;border-radius:3px;font-weight:600;` + badgeStyle;
        badge.textContent = dir.toUpperCase();
        const DIR_TIP = {
            input:  'input port — timed against arrival time. The diagram ' +
                'shows the arrival window (from set_input_delay) relative ' +
                'to the capturing clock edge.',
            output: 'output port — timed against departure. The diagram ' +
                'shows the required-stable window (from set_output_delay) ' +
                'before the downstream capture edge.',
            inout:  'bidirectional (inout) port — both arrival and ' +
                'departure are constrained. Each direction is shown on ' +
                'its own card.',
        };
        if (DIR_TIP[dir]) badge.title = DIR_TIP[dir];
        const portNameEl = document.createElement('span');
        portNameEl.style.cssText =
            'font-family:monospace;font-weight:600;color:var(--fg-primary);'
            + 'min-width:0;' + TRUNCATE_PATH_CSS;
        portNameEl.textContent = e0.port;
        // Mirror the Endpoints card: the name span carries a tooltip
        // with the full path so it's still readable when the column
        // truncates with text-overflow:ellipsis on narrow panes.
        portNameEl.title = e0.port;
        this._linkifyPin(portNameEl, e0, 'port');
        header.appendChild(badge);
        header.appendChild(portNameEl);

        // Tag the port with its constraint state.
        //   • Clock-source ports (where create_clock anchors) get a
        //     coloured "clock" pill so they aren't mistaken for
        //     unconstrained data ports — they're SUPPOSED to have no
        //     set_input_delay; the clock arrives via the create_clock
        //     waveform.
        //   • Otherwise, unconstrained rows (no input/output delay, no
        //     exception reference, no set_load / set_driving_cell) get
        //     a muted "unconstrained" tag so users can tell at a
        //     glance which IO has zero SDC coverage.
        if (e0.is_clock_port) {
            const tag = document.createElement('span');
            tag.style.cssText =
                'font-size:11px;padding:1px 6px;border-radius:8px;' +
                'background:var(--sdc-master-bg, rgba(80, 140, 220, 0.25));' +
                'color:var(--sdc-master-fg, var(--fg-primary));' +
                'font-weight:600;';
            tag.textContent = 'clock';
            tag.title =
                'Clock-source port — `create_clock` anchors on this port. ' +
                'No set_input_delay is expected; the clock waveform itself ' +
                'is the timing reference.';
            header.appendChild(tag);
        } else if (e0.is_unconstrained) {
            const tag = document.createElement('span');
            tag.style.cssText =
                'font-size:11px;padding:1px 6px;border-radius:8px;' +
                'background:var(--bg-input-deep);color:var(--fg-muted);' +
                'font-style:italic;';
            tag.textContent = 'unconstrained';
            tag.title =
                'No set_input_delay / set_output_delay, no exception ' +
                'reference, and no set_load / set_driving_cell on this ' +
                'port. STA will use defaults (zero arrival, ideal driver) ' +
                'when timing through it.';
            header.appendChild(tag);
        }

        // Driving cell, load/wire cap, and fanout (from first entry with data).
        // load_cap   : set_load              — pin capacitance
        // wire_cap   : set_load -wire        — net capacitance
        // fanout_load: set_fanout_load       — number of equivalent loads
        const anyEntry = portEntries.find(e =>
            e.driving_cell != null || e.load_cap != null
            || e.wire_cap != null  || e.fanout_load != null) || {};
        if (anyEntry.driving_cell != null) {
            const dcSpan = document.createElement('span');
            dcSpan.style.cssText = 'font-size:11px;color:var(--fg-muted);font-family:monospace;';
            const pin = anyEntry.driving_pin ? `/${anyEntry.driving_pin}` : '';
            dcSpan.textContent = `driving pin: ${anyEntry.driving_cell}${pin}`;
            dcSpan.title =
                'set_driving_cell — uses the named library cell\'s output ' +
                'characteristics (slew, drive resistance) when computing ' +
                'arrival on this port instead of an ideal source.';
            header.appendChild(dcSpan);
        }
        const capUnit = this._pdCapUnit || 'pF';
        const stat = (txt, tip) => {
            const s = document.createElement('span');
            s.style.cssText =
                'font-size:11px;color:var(--fg-muted);font-family:monospace;';
            s.textContent = txt;
            if (tip) s.title = tip;
            header.appendChild(s);
        };
        if (anyEntry.load_cap != null)
            stat(`load: ${anyEntry.load_cap.toPrecision(3)}${capUnit}`,
                 'set_load — pin capacitance');
        if (anyEntry.wire_cap != null)
            stat(`wire: ${anyEntry.wire_cap.toPrecision(3)}${capUnit}`,
                 'set_load -wire — net wire capacitance');
        if (anyEntry.fanout_load != null)
            stat(`fanout: ${anyEntry.fanout_load.toPrecision(3)}`,
                 'set_fanout_load — number of equivalent loads');
        // set_input_transition slews + set_drive resistance — both live on
        // the InputDrive object alongside set_driving_cell. Display the
        // common rise/max value as the headline; if rise/fall or min/max
        // disagree, append the asymmetric variant in parentheses so the
        // user can spot it without us emitting four columns of clutter.
        const tu = this._pdTimeUnit || 'ns';
        const sym = (rise, fall, suffix, label, tip) => {
            if (rise == null && fall == null) return;
            const fmt = (v) => `${v.toPrecision(3)}${suffix}`;
            // Same rise+fall → one number; diverging → "X / Y"
            const txt = (rise != null && fall != null && rise !== fall)
                ? `${label}: ${fmt(rise)} / ${fmt(fall)}`
                : `${label}: ${fmt(rise != null ? rise : fall)}`;
            stat(txt, tip);
        };
        // Take the "max" cell for the headline; min usually equals max for
        // these constraints (set_input_transition X without -min specifier).
        sym(anyEntry.drive_slew_rise_max, anyEntry.drive_slew_fall_max, tu,
            'in_slew',
            'set_input_transition — input slew applied to this port');
        sym(anyEntry.drive_res_rise_max,  anyEntry.drive_res_fall_max,  'Ω',
            'drive_res',
            'set_drive — output drive resistance applied to this port');
        // set_input_delay / set_output_delay flags + reference pin.
        // Surfaced when present so the user can tell whether a delay
        // value sits on top of the clock's network latency, or replaces
        // it (-source_latency_included / -network_latency_included), and
        // whether an alternate reference pin (-reference_pin) is in use.
        const flagStyle =
            'font-size:11px;font-style:italic;color:var(--fg-muted);' +
            'padding:0 4px;border:1px dotted var(--border);border-radius:2px;';
        if (e0.source_latency_included) {
            const f = document.createElement('span');
            f.style.cssText = flagStyle;
            f.textContent = '-source_latency_included';
            f.title = 'The delay value already includes the clock\'s source latency';
            header.appendChild(f);
        }
        if (e0.network_latency_included) {
            const f = document.createElement('span');
            f.style.cssText = flagStyle;
            f.textContent = '-network_latency_included';
            f.title = 'The delay value already includes the clock\'s network latency';
            header.appendChild(f);
        }
        if (e0.ref_pin) {
            const r = document.createElement('span');
            r.style.cssText =
                'font-size:11px;color:var(--fg-muted);font-family:monospace;';
            r.appendChild(document.createTextNode('ref_pin: '));
            const link = document.createElement('span');
            link.textContent = e0.ref_pin;
            link.title = e0.ref_pin;
            this._linkifyPin(link, e0, 'ref_pin');
            r.appendChild(link);
            header.appendChild(r);
        }
        wrapper.appendChild(header);

        // Render input-delay and output-delay entries separately.
        // Inout ports may have both; ordinary input/output ports have one.
        // When both are present, label each sub-section so the user knows
        // which orientation applies to which diagram. Exception-only
        // entries (no set_input/output_delay; the port shows up because
        // an exception references it) carry no delay/clock data and are
        // skipped here — the "Applicable Exceptions" toggle below covers
        // them.
        const inputEntries  = portEntries.filter(e => e.is_input  && !e.exception_only);
        const outputEntries = portEntries.filter(e => !e.is_input && !e.exception_only);
        const showSubLabels = inputEntries.length > 0 && outputEntries.length > 0;

        // Tooltip keyed by direction-section so the header reads as a
        // self-contained explanation of what's in the section. Mirrors
        // the per-pin sub-headers on Endpoints (Setup checks / Clock
        // pin / etc.) where the label always carries an explainer.
        const SECTION_TIP = {
            'Input delays':
                'set_input_delay constraints — arrival times relative ' +
                'to the capturing clock edge. Each diagram below shows ' +
                'one (clock, period) group; multi-launch -add_delay ' +
                'entries collapse onto a single shared waveform.',
            'Output delays':
                'set_output_delay constraints — required-stable window ' +
                'before the downstream capture edge. Each diagram below ' +
                'shows one (clock, period) group.',
        };
        const renderSection = (entries, label) => {
            if (entries.length === 0) return;
            if (showSubLabels) {
                const sub = document.createElement('div');
                sub.style.cssText =
                    'padding:3px 8px;font-size:11px;font-weight:600;' +
                    'color:var(--fg-muted);background:var(--bg-input-deep);' +
                    'border-bottom:1px solid var(--border-subtle);';
                sub.textContent = label;
                if (SECTION_TIP[label]) sub.title = SECTION_TIP[label];
                wrapper.appendChild(sub);
            }
            // Sub-group by (clock, period) so multiple constraints against
            // the same clock — e.g. set_input_delay -clock clk vs
            // set_input_delay -clock clk -clock_fall — collapse into ONE
            // multi-lane diagram with a shared clock waveform.
            const byClock = new Map();
            const noClock = [];
            for (const e of entries) {
                if (e.clock && e.clk_period > 0) {
                    const key = `${e.clock}|${e.clk_period}`;
                    if (!byClock.has(key)) byClock.set(key, []);
                    byClock.get(key).push(e);
                } else {
                    noClock.push(e);
                }
            }
            for (const [, group] of byClock) {
                const node = group.length === 1
                    ? this._renderPortDelayDiagram(group[0], timeUnit)
                    : this._renderPdMultiLane(group, timeUnit);
                if (node) wrapper.appendChild(node);
            }
            for (const entry of noClock) {
                const node = this._renderPortDelayDiagram(entry, timeUnit);
                if (node) wrapper.appendChild(node);
            }
        };
        renderSection(inputEntries,  'Input delays');
        renderSection(outputEntries, 'Output delays');

        // Collapsible "Applicable Exceptions" sub-section — closed by
        // default. The backend tags each entry with `exception_count`,
        // so we only attach the toggle when there's actually something
        // to show. Expanding lazy-fetches the port's full pin detail
        // via sdc_endpoint and renders the exception rows.
        const excCount = portEntries.reduce(
            (m, e) => Math.max(m, +(e.exception_count || 0)), 0);
        if (excCount > 0) {
            wrapper.appendChild(this._makePortDelayExceptionsToggle(
                portName, timeUnit, excCount));
        }

        return wrapper;
    }

    // Collapsible exception-list toggle for a Port Delays card. The
    // backend pre-counts how many exceptions reference this port (see
    // `exception_count` on each port_delays entry), so the toggle is
    // only created when count > 0. The actual exception list is still
    // lazy-fetched via sdc_endpoint on first ▶ click.
    _makePortDelayExceptionsToggle(portName, timeUnit, excCount) {
        const wrap = document.createElement('div');
        wrap.className = 'sdc-pd-exc-wrap';
        wrap.style.cssText =
            'border-top:1px solid var(--border-subtle);background:var(--bg-input);';

        const toggle = document.createElement('div');
        toggle.className = 'sdc-pd-exc-toggle';
        toggle.style.cssText =
            'display:flex;align-items:center;gap:6px;padding:3px 8px;' +
            'cursor:pointer;font-size:12px;color:var(--fg-muted);user-select:none;';
        const arrow = document.createElement('span');
        arrow.style.cssText =
            'font-size:11px;color:var(--fg-muted);width:9px;flex-shrink:0;';
        arrow.textContent = '▶';
        const label = document.createElement('span');
        label.textContent = `Applicable Exceptions (${excCount})`;
        toggle.appendChild(arrow);
        toggle.appendChild(label);
        wrap.appendChild(toggle);

        const body = document.createElement('div');
        body.className = 'sdc-pd-exc-body';
        body.style.cssText =
            'display:none;padding:4px 8px 6px 8px;flex-direction:column;gap:3px;';
        wrap.appendChild(body);

        let expanded = false;
        let loaded = false;
        toggle.addEventListener('click', async () => {
            expanded = !expanded;
            arrow.textContent = expanded ? '▼' : '▶';
            body.style.display = expanded ? 'flex' : 'none';
            if (expanded && !loaded) {
                loaded = true;
                const orig = label.textContent;
                label.textContent = `${orig} — loading…`;
                try {
                    const data = await this._requestWithTimeout({
                        type: 'sdc_endpoint',
                        pin:    portName,
                        offset: 0,
                        limit:  -1,
                    });
                    const pin = (data.pins || []).find(p => p.name === portName)
                              || (data.pins || [])[0];
                    const exc = (pin && pin.exceptions) || [];
                    label.textContent = `Applicable Exceptions (${exc.length})`;
                    body.innerHTML = '';
                    if (exc.length === 0) {
                        // Backend said count > 0 but the detail fetch
                        // returned none — schema mismatch, fail safe by
                        // showing a placeholder rather than silently empty.
                        const note = document.createElement('div');
                        note.style.cssText =
                            'font-size:12px;color:var(--fg-muted);font-style:italic;';
                        note.textContent = 'No exception detail available.';
                        body.appendChild(note);
                    } else {
                        for (const e of exc) {
                            body.appendChild(this._makeExcRow(e, timeUnit));
                        }
                    }
                } catch (err) {
                    label.textContent = `${orig} — error`;
                    body.innerHTML = '';
                    const errDiv = document.createElement('div');
                    errDiv.style.cssText =
                        'font-size:12px;color:var(--fg-muted);font-style:italic;';
                    errDiv.textContent = `${(err && err.message) || err}`;
                    body.appendChild(errDiv);
                }
            }
        });

        return wrap;
    }

    _updatePdFooter() {
        const loaded = this._pdAllEntries ? this._pdAllEntries.length : 0;
        const total  = this._pdTotal || loaded;
        if (total > loaded) {
            this._showPdMoreFooter(
                `Loaded ${loaded} of ${total} entries — scroll for more`);
        } else if (total > this._batchSize()) {
            this._showPdMoreFooter(`All ${total} entries loaded`);
        } else {
            // Remove footer if everything fits in one batch (no pagination needed).
            const f = this._pdScrollArea.querySelector('.sdc-pd-more-footer');
            if (f) f.remove();
        }
    }

    // Fallback for entries with no clock period: render a compact text summary
    // row so the constraint is still visible even without a timing diagram.
    _renderPortDelayFallback(entry, timeUnit) {
        const row = document.createElement('div');
        row.style.cssText =
            'display:flex;align-items:baseline;gap:12px;padding:5px 10px;' +
            'font-size:12px;color:var(--fg-secondary);border-top:1px solid var(--border-subtle);';

        const clkSpan = document.createElement('span');
        clkSpan.style.cssText = 'font-family:monospace;color:var(--fg-muted);';
        clkSpan.textContent = entry.clock ? `clk: ${entry.clock}` : 'clock: (undefined)';
        row.appendChild(clkSpan);

        const delays = [];
        if (entry.rise_max != null) delays.push(`rise_max=${entry.rise_max.toPrecision(3)}`);
        if (entry.rise_min != null) delays.push(`rise_min=${entry.rise_min.toPrecision(3)}`);
        if (entry.fall_max != null) delays.push(`fall_max=${entry.fall_max.toPrecision(3)}`);
        if (entry.fall_min != null) delays.push(`fall_min=${entry.fall_min.toPrecision(3)}`);
        if (delays.length === 0)    delays.push('(no delay values)');

        const valSpan = document.createElement('span');
        valSpan.style.fontFamily = 'monospace';
        valSpan.textContent = delays.join('  ') + (timeUnit ? ` ${timeUnit}` : '');
        row.appendChild(valSpan);

        const note = document.createElement('span');
        note.style.cssText = 'font-style:italic;color:var(--fg-muted);';
        note.textContent = '(no clock period — diagram unavailable)';
        row.appendChild(note);

        return row;
    }

    // ── Exceptions panel ────────────────────────────────────────────────────

    _buildExceptionsPanel(container) {
        container.style.cssText = 'display:flex;flex-direction:column;height:100%;overflow:hidden;';

        const toolbar = document.createElement('div');
        toolbar.style.cssText =
            'display:flex;align-items:center;padding:2px 6px;gap:6px;border-bottom:1px solid var(--border);' +
            'background:var(--bg-header);flex-shrink:0;';

        const refreshBtn = document.createElement('button');
        refreshBtn.textContent = '↺ Refresh';
        refreshBtn.title = 'Reload timing exceptions from the current design';
        refreshBtn.style.cssText =
            'padding:2px 8px;font-size:12px;cursor:pointer;background:var(--bg-input);' +
            'color:var(--fg-primary);border:1px solid var(--border);border-radius:3px;';
        refreshBtn.addEventListener('click', () => {
            this._excLoaded = false;
            this._excLoading = false;
            this._loadModes();
            this._loadExceptions();
        });
        toolbar.appendChild(refreshBtn);

        // Type filter buttons
        this._excActiveFilter = 'all';
        this._excFilterBtns = this._makeFilterButtons({
            container: toolbar,
            defs: [
                { key: 'all',         label: 'All' },
                { key: 'false_path',  label: 'false path' },
                { key: 'multi_cycle', label: 'multi cycle' },
                { key: 'path_delay',  label: 'path delay' },
                { key: 'group_path',  label: 'group path' },
            ],
            initialKey: 'all',
            datasetPrefix: 'excFilter',
            onSelect: (key) => this._setExcFilter(key),
        });

        container.appendChild(toolbar);

        const scroll = document.createElement('div');
        scroll.style.cssText = 'flex:1;overflow-y:auto;padding:8px;background:var(--bg-main);';
        this._excScrollArea = scroll;
        container.appendChild(scroll);

        scroll.innerHTML =
            '<div style="padding:12px;color:var(--fg-muted);font-style:italic;">Loading…</div>';
    }

    // Button styling is handled by the _makeFilterButtons helper; this
    // method just owns the state transition + re-render so the trailing
    // `_setExcFilter('all')` init call still works.
    _setExcFilter(filter) {
        this._excActiveFilter = filter;
        if (this._excLoaded) this._applyExcFilter();
    }

    async _loadExceptions() {
        if (this._excLoaded || this._excLoading) return;
        this._excLoading = true;
        this._excScrollArea.innerHTML =
            '<div style="padding:12px;color:var(--fg-muted);font-style:italic;">Loading…</div>';
        try {
            await this._app.websocketManager.readyPromise;
            const data = await this._requestWithTimeout({ type: 'sdc_exceptions' });
            this._excData = data;
            this._excLoaded = true;
            this._applyExcFilter();
        } catch (e) {
            console.error('[SDC] exceptions load error:', e);
            this._showLoadError(this._excScrollArea, 'exceptions', e, () => {
                this._excLoaded = false;
                this._excLoading = false;
                this._loadExceptions();
            });
        } finally {
            this._excLoading = false;
        }
    }

    _applyExcFilter() {
        const data = this._excData || { exceptions: [] };
        const f = this._excActiveFilter;
        this._updateExcFilterCounts();
        const filtered = f === 'all'
            ? data.exceptions
            : data.exceptions.filter(e => e.type === f);
        this._renderExceptions(filtered, data.time_unit || 'ns');
    }

    // Append running counts to the exception type-filter buttons so the
    // user sees how many exceptions land in each bucket. Mirrors the
    // count style used on the Endpoints kind toolbar.
    _updateExcFilterCounts() {
        if (!this._excFilterBtns) return;
        const all = (this._excData && this._excData.exceptions) || [];
        const counts = { all: all.length };
        for (const e of all) {
            if (!e || !e.type) continue;
            counts[e.type] = (counts[e.type] || 0) + 1;
        }
        for (const [key, btn] of Object.entries(this._excFilterBtns)) {
            const lbl = btn.dataset.excFilterLabel || btn.textContent;
            const n = counts[key] || 0;
            btn.textContent = `${lbl} (${n})`;
        }
    }

    _renderExceptions(exceptions, timeUnit) {
        this._excScrollArea.innerHTML = '';

        if (exceptions.length === 0) {
            this._excScrollArea.innerHTML =
                '<div style="padding:12px;color:var(--fg-muted);font-style:italic;">' +
                'No timing exceptions match the current filter.</div>';
            return;
        }

        // Summary count bar
        const summary = document.createElement('div');
        summary.style.cssText =
            'padding:4px 8px;font-size:12px;color:var(--fg-muted);' +
            'border-bottom:1px solid var(--border-subtle);margin-bottom:8px;';
        summary.textContent = `${exceptions.length} exception${exceptions.length !== 1 ? 's' : ''}`;
        this._excScrollArea.appendChild(summary);

        for (const exc of exceptions) {
            this._excScrollArea.appendChild(this._makeExcRow(exc, timeUnit));
        }
    }

    _makeExcRow(exc, timeUnit) {
        const row = document.createElement('div');
        row.style.cssText =
            'margin-bottom:6px;border:1px solid var(--border);border-radius:4px;' +
            'overflow:hidden;font-size:12px;';

        // Header: type badge + value annotation
        const header = document.createElement('div');
        header.style.cssText =
            'display:flex;align-items:center;gap:8px;padding:4px 8px;' +
            'background:var(--bg-header);border-bottom:1px solid var(--border-subtle);';

        // Type badge. Carries a data-exc-type attribute (and -multiplier /
        // -delay where applicable) so tests can assert on structure rather
        // than exact textContent / Unicode glyphs.
        const badge = document.createElement('span');
        const badgeStyles = {
            false_path:  'background:var(--sdc-false-bg);color:var(--sdc-false-fg);',
            multi_cycle: 'background:var(--sdc-mcp-bg);color:var(--sdc-mcp-fg);',
            path_delay:  'background:var(--sdc-pd-bg);color:var(--sdc-pd-fg);',
            group_path:  'background:var(--sdc-group-bg);color:var(--sdc-group-fg);',
        };
        badge.style.cssText =
            `font-size:11px;padding:1px 6px;border-radius:3px;font-weight:600;white-space:nowrap;` +
            (badgeStyles[exc.type] || 'background:var(--bg-input);color:var(--fg-muted);');
        badge.dataset.excType = exc.type;
        if (exc.type === 'multi_cycle' && exc.multiplier != null) {
            badge.dataset.multiplier = String(exc.multiplier);
        }
        if (exc.type === 'path_delay' && exc.delay != null) {
            badge.dataset.delay = String(exc.delay);
        }
        // Build a self-explanatory badge text. For MCP we fold the scope
        // (setup / hold / setup+hold) and the -end flag into the badge
        // itself instead of leaving them as separate tags downstream —
        // "MCP ×2" alone doesn't say which check it's relaxing.
        const mmText = exc.min_max === 'max' ? 'setup'
                     : exc.min_max === 'min' ? 'hold'
                     : 'setup+hold';
        if (exc.type === 'false_path') {
            badge.textContent = 'FALSE PATH';
            badge.title = 'set_false_path — matching paths are excluded from timing analysis';
        } else if (exc.type === 'multi_cycle') {
            const ref = exc.use_end_clk ? '  (cap-relative)' : '';
            badge.textContent = `MCP ${mmText} ×${exc.multiplier}${ref}`;
            const cmdParts = [`set_multicycle_path ${exc.multiplier}`];
            if (exc.min_max === 'max') cmdParts.push('-setup');
            else if (exc.min_max === 'min') cmdParts.push('-hold');
            // 'all' means setup+hold — no flag (default)
            if (exc.use_end_clk) cmdParts.push('-end');
            const refDesc = exc.use_end_clk
                ? `${exc.multiplier} cycles after the launch edge measured at the capture clock.`
                : `${exc.multiplier} cycles after the launch edge measured at the launch clock.`;
            const checkDesc = exc.min_max === 'max'
                ? 'Setup check uses '
                : exc.min_max === 'min'
                ? 'Hold check uses '
                : 'Setup and hold checks both use ';
            badge.title = `${cmdParts.join(' ')}\n${checkDesc}${refDesc}`;
        } else if (exc.type === 'path_delay') {
            const v = exc.delay != null ? exc.delay.toPrecision(3) + timeUnit : '';
            badge.textContent = `PATH DELAY ${mmText} ${v}`.trimEnd();
            badge.title =
                `set_${exc.min_max === 'min' ? 'min' : 'max'}_delay ${v} — ` +
                'overrides the default clock-derived path bound for matching paths';
        } else if (exc.type === 'group_path') {
            badge.textContent = `GROUP${exc.is_default ? ' (default)' : ''}`;
            badge.title = 'set_group_path — collect matching paths into a named report group';
        } else {
            badge.textContent = exc.type;
        }
        header.appendChild(badge);

        // Group path name (only for group_path rows)
        if (exc.type === 'group_path' && exc.name) {
            const nameSpan = document.createElement('span');
            nameSpan.style.cssText =
                'font-family:monospace;font-size:12px;color:var(--fg-primary);font-weight:600;';
            nameSpan.textContent = exc.name;
            header.appendChild(nameSpan);
        }

        // min/max scope — folded into the badge for multi_cycle and
        // path_delay (where it's part of the SDC command name); shown as
        // a separate label only for false_path / group_path where it
        // actually adds information.
        if (exc.type !== 'multi_cycle' && exc.type !== 'path_delay') {
            const mmSpan = document.createElement('span');
            mmSpan.style.cssText = 'font-size:11px;color:var(--fg-muted);';
            mmSpan.textContent = mmText;
            header.appendChild(mmSpan);
        }

        // path_delay flags — only meaningful on path_delay rows.  Each one
        // gets a small italic tag so the exception's command-line surface
        // is fully visible without clicking through.
        const flagStyle =
            'font-size:11px;font-style:italic;color:var(--fg-muted);' +
            'padding:0 4px;border:1px dotted var(--border);border-radius:2px;';
        if (exc.type === 'path_delay') {
            if (exc.ignore_clk_latency) {
                const f = document.createElement('span');
                f.style.cssText = flagStyle;
                f.textContent = '-ignore_clock_latency';
                f.title = 'Path delay computed without including clock latency';
                header.appendChild(f);
            }
            if (exc.break_path) {
                const f = document.createElement('span');
                f.style.cssText = flagStyle;
                f.textContent = '-break_path';
                f.title = 'Treat the constrained path as a clean break (paths upstream/downstream are not extended)';
                header.appendChild(f);
            }
        }
        // multi_cycle -end is now folded into the MCP badge as
        // "(cap-relative)", so we no longer emit a separate italic tag.

        row.appendChild(header);

        // Body: from / thru / to
        const body = document.createElement('div');
        body.style.cssText = 'padding:4px 8px;display:flex;flex-direction:column;gap:3px;';

        // Render each endpoint (from / thru / to) as its own block with a
        // stacked list of pins and clocks.  For exceptions that cover many
        // endpoints (hundreds of pins on a false_path is common) a single
        // comma-joined line pushes the value off-screen; wrapping rows with
        // individual entries lets the user actually read them.
        const addEndpoint = (label, pins, clocks, opts) => {
            opts = opts || {};
            const insts = opts.insts || [];
            const nets  = opts.nets  || [];
            const transition = opts.transition;  // 'rise' | 'fall' | 'rise_fall' | null
            if ((!pins || pins.length === 0)
                && (!clocks || clocks.length === 0)
                && (!insts || insts.length === 0)
                && (!nets  || nets.length  === 0)) return;
            const block = document.createElement('div');
            block.style.cssText = 'display:flex;gap:6px;align-items:flex-start;';

            // Label is the SDC clause keyword. When the exception specifies
            // a transition (-rise_from, -fall_to, etc.), append it so users
            // can see the edge restriction at a glance: "-FROM (rise)".
            const lbl = document.createElement('span');
            lbl.style.cssText =
                'flex-shrink:0;min-width:40px;padding-top:1px;font-size:11px;font-weight:600;' +
                'color:var(--fg-muted);text-transform:uppercase;letter-spacing:0.04em;';
            const edgeSym = transition === 'rise' ? ' ↑'
                          : transition === 'fall' ? ' ↓' : '';
            lbl.textContent = label + edgeSym;
            if (transition === 'rise')
                lbl.title = `${label} restricted to rising-edge transitions`;
            else if (transition === 'fall')
                lbl.title = `${label} restricted to falling-edge transitions`;
            block.appendChild(lbl);

            // Wide exceptions can have hundreds of pins on a single false_path.
            // Strategy: progressive disclosure — show the first INITIAL_VISIBLE
            // entries up front, then a "+ N more" expander.  Clicking it
            // reveals the rest in-place inside a scrollable container so the
            // row never dominates the page but every entry is reachable.
            const INITIAL_VISIBLE = 20;
            // Use plain block layout (no flex) so each row stacks by default,
            // independent of any flex-item quirks.  min-width:0 lets the
            // ellipsis truncation kick in when a hierarchical pin path is
            // wider than the column.
            const list = document.createElement('div');
            list.style.cssText = 'flex:1;min-width:0;';

            // Each entry is a div with explicit line-height + padding so rows
            // never visually collide regardless of the inherited line-height.
            const itemStyle =
                'font-family:monospace;font-size:12px;line-height:1.4;' +
                'padding:1px 0;color:var(--fg-primary);' +
                'overflow:hidden;text-overflow:ellipsis;white-space:nowrap;';

            // Flat sequence so the initial-visible window can span clocks +
            // pins + instances + nets uniformly. Each kind gets a small
            // icon prefix so users can tell at a glance whether the entry
            // is a clock, pin, instance, or net.
            const all = [
                ...((clocks || []).map(c => ({ kind: 'clock', payload: c }))),
                ...((pins   || []).map(p => ({ kind: 'pin',
                    payload: typeof p === 'string' ? { name: p } : p }))),
                ...((insts  || []).map(i => ({ kind: 'inst',
                    payload: typeof i === 'string' ? { name: i } : i }))),
                ...((nets   || []).map(n => ({ kind: 'net',
                    payload: typeof n === 'string' ? { name: n } : n }))),
            ];

            const renderItem = (entry) => {
                const item = document.createElement('div');
                item.style.cssText = itemStyle;
                if (entry.kind === 'clock') {
                    item.title = entry.payload;
                    item.textContent = `⏱ ${entry.payload}`;
                } else if (entry.kind === 'inst') {
                    item.title = `instance: ${entry.payload.name}`;
                    item.textContent = `▣ ${entry.payload.name}`;
                    // Instances reuse _linkifyPin (it dispatches on odb_type).
                    this._linkifyPin(item, entry.payload);
                } else if (entry.kind === 'net') {
                    item.title = `net: ${entry.payload.name}`;
                    item.textContent = `─ ${entry.payload.name}`;
                } else {
                    item.title = entry.payload.name;
                    item.textContent = entry.payload.name;
                    this._linkifyPin(item, entry.payload);
                }
                return item;
            };

            const head = all.slice(0, INITIAL_VISIBLE);
            const tail = all.slice(INITIAL_VISIBLE);

            for (const e of head) list.appendChild(renderItem(e));

            if (tail.length > 0) {
                // Hidden container; revealed by the expander.  display:block
                // (vs. flex) keeps the toggle a simple `none` ↔ `block` flip
                // without depending on flex semantics — earlier flex version
                // ran into a visibility issue where toggled-on content
                // appeared but rendered at zero height for some users.
                const more = document.createElement('div');
                more.style.cssText =
                    'display:none;' +
                    (tail.length > 60 ? 'max-height:240px;overflow-y:auto;' : '');
                for (const e of tail) more.appendChild(renderItem(e));

                const expander = document.createElement('button');
                expander.style.cssText =
                    'display:inline-block;margin-top:4px;padding:1px 6px;' +
                    'font-size:11px;cursor:pointer;background:var(--bg-input);' +
                    'color:var(--accent-link);border:1px solid var(--border);' +
                    'border-radius:3px;font-family:inherit;';
                let expanded = false;
                const updateExpander = () => {
                    expander.textContent = expanded
                        ? `– hide ${tail.length} more`
                        : `+ ${tail.length} more`;
                    more.style.display = expanded ? 'block' : 'none';
                };
                updateExpander();
                expander.addEventListener('click', (e) => {
                    e.preventDefault();
                    e.stopPropagation();
                    expanded = !expanded;
                    updateExpander();
                });
                list.appendChild(expander);
                list.appendChild(more);
            }
            block.appendChild(list);
            body.appendChild(block);
        };

        addEndpoint('-from', exc.from_pins, exc.from_clocks, {
            insts: exc.from_insts,
            transition: exc.from_transition,
        });
        if (exc.thrus) {
            exc.thrus.forEach((thru, i) => {
                addEndpoint(`-thru${exc.thrus.length > 1 ? i + 1 : ''}`,
                            thru.pins, thru.clocks, {
                                insts: thru.insts,
                                nets:  thru.nets,
                                transition: thru.transition,
                            });
            });
        }
        addEndpoint('-to', exc.to_pins, exc.to_clocks, {
            insts: exc.to_insts,
            transition: exc.to_transition,
        });

        if (body.children.length === 0) {
            const any = document.createElement('span');
            any.style.cssText = 'color:var(--fg-muted);font-style:italic;font-size:12px;';
            any.textContent = '(applies to all paths)';
            body.appendChild(any);
        }

        row.appendChild(body);
        // SDC -comment from set_false_path / set_multicycle_path / etc.
        this._appendComment(row, exc.comment);
        return row;
    }

    // Draw a single timing diagram for one port-delay entry.
    // Shows: clock waveform + data valid window with constraint annotation.
    // If clk_period is absent (clock undefined), renders a text-only fallback row.
    _renderPortDelayDiagram(entry, timeUnit) {
        const T = entry.clk_period;
        if (!T || T <= 0) {
            return this._renderPortDelayFallback(entry, timeUnit);
        }

        // Per-transition constraint sets. Each is null when that transition
        // is unconstrained, otherwise {min, max, hasSpread}. set_input_delay
        // lets the user constrain rise and fall independently — when they
        // differ, both arrivals are drawn on a single bar (color-coded).
        const riseSet = this._makeDelaySet(entry.rise_max, entry.rise_min);
        const fallSet = this._makeDelaySet(entry.fall_max, entry.fall_min);
        if (!riseSet && !fallSet) {
            return this._renderPortDelayFallback(entry, timeUnit);
        }

        // Equal sets collapse to a single marker — same visual as the
        // pre-combined rendering (and the common SDC case where the user
        // didn't pass -rise/-fall flags so both edges share one constraint).
        const transitionsEqual = riseSet && fallSet
            && riseSet.min === fallSet.min && riseSet.max === fallSet.max;
        const showBoth = riseSet && fallSet && !transitionsEqual;

        // Combined union drives the bar zones (invalid / hold_unc / setup_unc / valid).
        // dMax = worst-case "latest" arrival → drives setup margin.
        // dMin = worst-case "earliest" arrival → drives hold margin.
        const allMax = [riseSet, fallSet].filter(s => s).map(s => s.max);
        const allMin = [riseSet, fallSet].filter(s => s).map(s => s.min);
        const dMax = Math.max(...allMax);
        const dMin = Math.min(...allMin);

        const LABEL_W = 200;
        const WAVE_W = Math.max(260, this._pdDiagramContainerW() - LABEL_W);
        // Layout: legend strip → CLK row → gap → DATA bar → axis → 2-row tick labels
        const { LEG_Y, CLK_HIGH, CLK_LOW, CLK_MID } = DIAGRAM_CONST;
        const DATA_TOP = 52;
        const DATA_BOT = 72;
        const AXIS_Y   = 82;
        const TICK_ROW = [AXIS_Y + 11, AXIS_Y + 23];  // two stagger rows for labels
        const SVG_H    = AXIS_Y + 34;

        // Map time in period units → x pixel within the wave area.
        const tx = (t) => LABEL_W + (t / T) * WAVE_W;

        const svg = this._makeSvg(LABEL_W + WAVE_W, SVG_H);

        // Clock uncertainty values needed throughout the diagram.
        const su = entry.uncertainty_setup || 0;
        const hu = entry.uncertainty_hold  || 0;
        const bothDiffer = dMin != null && dMax != null && dMin !== dMax;

        // ── Left label area ──────────────────────────────────────────────────
        const edgeStr = entry.clk_edge === 'rise' ? '↑'
                      : entry.clk_edge === 'fall' ? '↓' : '↑↓';
        const clkLabel = entry.clock ? `${entry.clock}${edgeStr}` : '(any)';
        this._svgText(svg, 4, CLK_MID + 4, 'CLK', 'var(--fg-muted)', 9, 'start');
        this._svgText(svg, 4, DATA_TOP + 9,  clkLabel, 'var(--canvas-label)', 9, 'start');

        // Delay value annotation: hold-adjusted min, nominal arrival/output, setup-adjusted max.
        // When rise/fall differ (showBoth), append a second line breaking out
        // the per-edge values so the reader can tie each marker back to a
        // numeric constraint.
        let annot = '';
        let annot2 = '';
        if (entry.is_input) {
            const nomArr    = dMax;
            const hasSpread = bothDiffer;
            const arrLeft   = hasSpread ? dMin : nomArr;
            const effMin = hu > 0 ? Math.max(0, arrLeft - hu) : arrLeft;
            const effMax = su > 0 ? Math.min(T, nomArr + su)  : nomArr;
            const typ    = hasSpread ? `${dMin.toPrecision(3)}–${nomArr.toPrecision(3)}` : nomArr.toPrecision(3);
            if (su > 0 || hu > 0) {
                annot = `min: ${effMin.toPrecision(3)}${timeUnit}  arr: ${typ}${timeUnit}  max: ${effMax.toPrecision(3)}${timeUnit}`;
            } else {
                annot = hasSpread ? `arr: ${typ}${timeUnit}` : `delay: ${nomArr.toPrecision(3)}${timeUnit}`;
            }
            if (showBoth) {
                const fmt = (s) => s.hasSpread
                    ? `${s.min.toPrecision(3)}–${s.max.toPrecision(3)}`
                    : s.max.toPrecision(3);
                annot2 = `↑ rise: ${fmt(riseSet)}${timeUnit}   ↓ fall: ${fmt(fallSet)}${timeUnit}`;
            }
        } else {
            const tOut    = T - dMax;
            const tChange = bothDiffer ? T - dMin : tOut;
            const hasSpread = bothDiffer;
            const effMin = su > 0 ? Math.max(0, tOut - su)    : tOut;
            const effMax = hu > 0 ? Math.min(T, tChange + hu) : tChange;
            // Show as negative values relative to the capture edge: e.g. set_output_delay 2 → "-2.00ns"
            const rel = (abs) => (abs - T);
            const negOut    = rel(tOut);
            const negChange = rel(tChange);
            const negEffMin = rel(effMin);
            const negEffMax = rel(effMax);
            const typ = hasSpread
                ? `${negOut.toPrecision(3)} to ${negChange.toPrecision(3)}`
                : negOut.toPrecision(3);
            if (su > 0 || hu > 0) {
                annot = `min: ${negEffMin.toPrecision(3)}${timeUnit}  out: ${typ}${timeUnit}  max: ${negEffMax.toPrecision(3)}${timeUnit}`;
            } else {
                annot = hasSpread ? `out: ${typ}${timeUnit}` : `output: ${negOut.toPrecision(3)}${timeUnit}`;
            }
            if (showBoth) {
                const fmt = (s) => s.hasSpread
                    ? `${(-s.max).toPrecision(3)} to ${(-s.min).toPrecision(3)}`
                    : (-s.max).toPrecision(3);
                annot2 = `↑ rise: ${fmt(riseSet)}${timeUnit}   ↓ fall: ${fmt(fallSet)}${timeUnit}`;
            }
        }
        if (annot) {
            this._svgText(svg, 4, DATA_BOT + 10, annot, 'var(--canvas-label)', 8, 'start');
        }
        if (annot2) {
            this._svgText(svg, 4, DATA_BOT + 21, annot2, 'var(--canvas-label)', 8, 'start');
        }

        // ── Clock waveform ───────────────────────────────────────────────────
        const isRiseRef = entry.clk_edge !== 'fall';

        // Uncertainty bands are always inside [0,T]; just add a small post-T tail.
        const PRE  = T * 0.15;
        const POST = T * 0.08;
        const TOTAL = T + PRE + POST;
        const txFull = (t) => LABEL_W + ((t + PRE) / TOTAL) * WAVE_W;

        // 50%-duty clock with launch (t=0), mid-period flip (t=T/2), and
        // capture (t=T). startLow encodes whether the path begins in the LOW
        // state (rise launch) or HIGH state (fall launch).
        const startLow = isRiseRef;
        const edges = [
            { t: 0,     isRise:  startLow },
            { t: T / 2, isRise: !startLow },
            { t: T,     isRise:  startLow },
        ];
        const startY = startLow ? CLK_LOW : CLK_HIGH;
        const clkPath = this._drawClockPath(
            svg, edges, txFull, CLK_HIGH, CLK_LOW, startY,
            -PRE, T + POST, 'var(--canvas-axis)', '1.5');
        // Hover the waveform itself to see clock summary.
        if (clkPath) {
            const edgeWord = entry.clk_edge === 'fall' ? 'falling'
                           : entry.clk_edge === 'rise' ? 'rising' : 'rising/falling';
            this._svgTitle(clkPath,
                `${entry.clock || '(virtual)'} — period ${T.toPrecision(3)}`
                + `${timeUnit}, capture on the ${edgeWord} edge at t=T.`);
        }

        // Dashed reference lines at launch (t=0) and capture (t=T)
        for (const refT of [0, T]) {
            const rx = txFull(refT);
            const line = this._svgLine(svg, rx, CLK_HIGH, rx, AXIS_Y,
                                       'var(--border-subtle)', '1');
            line.setAttribute('stroke-dasharray', '3,3');
            line.dataset.ref = refT === 0 ? 'launch' : 'capture';
            this._svgTitle(line, refT === 0
                ? 'launch edge — t=0 of the period; the upstream flop '
                  + 'drives data here on this edge.'
                : `capture edge — t=T (${T.toPrecision(3)}${timeUnit}); the `
                  + 'downstream flop samples data here.');
        }

        // ── Data bar ─────────────────────────────────────────────────────────
        // Bands paint as theme-aware CSS-var fills via BAND_COLORS. 0.85
        // opacity keeps a small amount of page-bg bleed-through (so axis/grid
        // overlays stay visible) while still reading as a solid tint. Local
        // COL_* aliases are retained because the legend code below references
        // them by color.
        const COL_INVALID   = BAND_COLORS['invalid'];
        const COL_UNCERT    = BAND_COLORS['uncert'];
        const COL_VALID     = BAND_COLORS['valid'];
        const COL_SETUP_UNC = BAND_COLORS['setup-unc'];
        const COL_HOLD_UNC  = BAND_COLORS['hold-unc'];
        const { BAR_OPACITY } = DIAGRAM_CONST;
        // Hover tooltip on every band so users can decode the colors
        // without leaning on the legend strip below — same descriptions
        // the legend uses (BAND_TIPS at module scope).
        const tipForKind = BAND_TIPS[entry.is_input ? 'input' : 'output'];
        const drawRect = (x1, x2, kind, opacity = BAR_OPACITY) => {
            const r = this._svgBandRect(svg, x1, DATA_TOP, x2 - x1,
                                        DATA_BOT - DATA_TOP, kind, opacity);
            if (r && tipForKind[kind]) this._svgTitle(r, tipForKind[kind]);
            return r;
        };

        const xCapture = txFull(T);

        if (entry.is_input) {
            // Margins bracket the arrival point or spread [dMin, dMax]:
            //   invalid   [0, arrLeft-hu]        — not yet arrived
            //   hold_unc  [arrLeft-hu, arrLeft]  — hold margin (arrLeft = dMin if spread, else dMax)
            //   spread    [dMin, dMax]            — amber (drawn only for single-transition spread)
            //   setup_unc [arrRight, arrRight+su] — setup margin (arrRight = dMax always)
            //   valid     [arrRight+su, T]
            // When showBoth is true, we skip the amber spread band so each
            // transition's own marker (rise/fall colored) reads cleanly.
            const x0 = txFull(0);
            const nomArr    = dMax;
            const hasSpread = bothDiffer;
            const arrLeft   = hasSpread ? dMin : nomArr;
            const arrRight  = nomArr;

            const holdStartAbs = arrLeft  - hu;   // may be < 0
            const setupEndAbs  = arrRight + su;   // may be > T
            const tHoldLeft = Math.max(0, holdStartAbs);
            const tSetupEnd = Math.min(T, setupEndAbs);
            drawRect(x0, txFull(tHoldLeft), 'invalid');
            if (hu > 0) drawRect(txFull(tHoldLeft), txFull(arrLeft), 'hold-unc');
            if (hasSpread && !showBoth) {
                drawRect(txFull(arrLeft), txFull(arrRight), 'uncert');
            }
            if (su > 0) drawRect(txFull(arrRight), txFull(tSetupEnd), 'setup-unc');
            drawRect(txFull(tSetupEnd), xCapture, 'valid');
            // Cyclic wrap: bands that extend past the period boundary also
            // appear at the other end (the next/previous period's launch).
            // Drawn last so they overlay the trailing/leading default fills.
            if (hu > 0 && holdStartAbs < 0) {
                drawRect(txFull(T + holdStartAbs), txFull(T), 'hold-unc');
            }
            if (su > 0 && setupEndAbs > T) {
                drawRect(txFull(0), txFull(setupEndAbs - T), 'setup-unc');
            }

            // Arrival markers — choose visual based on which transitions
            // carry constraints, so the user can tell at a glance:
            //   • only rise  → yellow line  + ↑
            //   • only fall  → pink line    + ↓
            //   • both equal → split marker + ↑↓
            //   • both differ→ two markers (one each)
            if (showBoth) {
                this._drawPdEdgeMarker(svg, txFull, riseSet, DATA_TOP, DATA_BOT,
                                       'var(--sdc-wf-marker-fill)', '↑');
                this._drawPdEdgeMarker(svg, txFull, fallSet, DATA_TOP, DATA_BOT,
                                       'var(--sdc-wf-fall-fill)',   '↓');
            } else if (!hasSpread) {
                this._drawPdSinglePoint(svg, txFull(nomArr), DATA_TOP, DATA_BOT,
                                        riseSet, fallSet);
            }

        } else {
            // Output margins bracket the output point or spread [T-dMax, T-dMin]:
            //   valid     [0, tOut-su]         — stable, clear of setup deadline
            //   setup_unc [tOut-su, tOut]       — setup margin (tOut = T-dMax, left anchor)
            //   spread    [tOut, tChange]        — amber (drawn only for single-transition spread)
            //   hold_unc  [tChange, tChange+hu] — hold margin (tChange = T-dMin, right anchor)
            //   invalid   [tChange+hu, T]
            const x0    = txFull(0);
            const tOut    = T - dMax;
            const tChange = bothDiffer ? T - dMin : tOut;
            const hasSpread = bothDiffer;

            const validEndAbs = tOut    - su;   // may be < 0
            const holdEndAbs  = tChange + hu;   // may be > T
            const tValidEnd = Math.max(0, validEndAbs);
            const tHoldEnd  = Math.min(T, holdEndAbs);
            drawRect(x0, txFull(tValidEnd), 'valid');
            if (su > 0) drawRect(txFull(tValidEnd), txFull(tOut), 'setup-unc');
            if (hasSpread && !showBoth) {
                drawRect(txFull(tOut), txFull(tChange), 'uncert');
            }
            if (hu > 0) drawRect(txFull(tChange), txFull(tHoldEnd), 'hold-unc');
            drawRect(txFull(tHoldEnd), xCapture, 'invalid');
            // Cyclic wrap: setup band extending before t=0 reappears at the
            // end of the period; hold band past t=T reappears at the start.
            if (su > 0 && validEndAbs < 0) {
                drawRect(txFull(T + validEndAbs), txFull(T), 'setup-unc');
            }
            if (hu > 0 && holdEndAbs > T) {
                drawRect(txFull(0), txFull(holdEndAbs - T), 'hold-unc');
            }

            // Output markers — same four-case visual (mirrored).
            // For output, the time-axis is mirrored (output point = T - dmax),
            // so we synthesize per-transition sets in T-domain coordinates.
            if (showBoth) {
                const mirror = (s) => ({
                    min: T - s.max,
                    max: T - s.min,
                    hasSpread: s.hasSpread,
                });
                this._drawPdEdgeMarker(svg, txFull, mirror(riseSet), DATA_TOP, DATA_BOT,
                                       'var(--sdc-wf-marker-fill)', '↑');
                this._drawPdEdgeMarker(svg, txFull, mirror(fallSet), DATA_TOP, DATA_BOT,
                                       'var(--sdc-wf-fall-fill)',   '↓');
            } else if (!hasSpread) {
                this._drawPdSinglePoint(svg, txFull(tOut), DATA_TOP, DATA_BOT,
                                        riseSet, fallSet);
            }
        }

        // Bar outline — spans [0, T]; uncertainty bands stay inside.
        const xBarStart = txFull(0);
        const xBarEnd   = xCapture;
        this._svgOutlineRect(svg, xBarStart, DATA_TOP,
                             xBarEnd - xBarStart, DATA_BOT - DATA_TOP);

        // ── Axis ─────────────────────────────────────────────────────────────
        this._svgLine(svg, xBarStart, AXIS_Y, xBarEnd, AXIS_Y,
                      'var(--canvas-axis)', 1);

        // Two-row tick layout:
        //   Row 0 (top)    = boundary values: uncertainty-adjusted min/max edges
        //   Row 1 (bottom) = nominal values:  0, raw arrival/output times, T
        // No (±Su/Hu) suffixes — the legend already explains the colours.
        const pendingTicks = [];
        const addTick = (t, label, row) => {
            const x = Math.max(LABEL_W + 1, Math.min(LABEL_W + WAVE_W - 1, txFull(t)));
            pendingTicks.push({ x, label, row });
        };

        // Nominal row
        addTick(0,  '0',                              1);
        addTick(T,  `T=${T.toPrecision(3)}${timeUnit}`, 1);

        if (entry.is_input) {
            const nomArr    = dMax;
            const hasSpread = bothDiffer;
            const arrLeft   = hasSpread ? dMin : nomArr;
            // Boundary row (always drawn from the union span)
            if (hu > 0)
                addTick(Math.max(0, arrLeft - hu), `${Math.max(0, arrLeft - hu).toPrecision(3)}`, 0);
            if (su > 0)
                addTick(Math.min(T, nomArr + su),  `${Math.min(T, nomArr + su).toPrecision(3)}`,  0);
            // Nominal row
            if (showBoth) {
                // Per-edge ticks — riseSet.max and fallSet.max (and mins if spread).
                addTick(riseSet.max, `${riseSet.max.toPrecision(3)}`, 1);
                addTick(fallSet.max, `${fallSet.max.toPrecision(3)}`, 1);
                if (riseSet.hasSpread) addTick(riseSet.min, `${riseSet.min.toPrecision(3)}`, 0);
                if (fallSet.hasSpread) addTick(fallSet.min, `${fallSet.min.toPrecision(3)}`, 0);
            } else {
                if (hasSpread) addTick(dMin, `${dMin.toPrecision(3)}`, 1);
                addTick(dMax, `${dMax.toPrecision(3)}`, 1);
            }
        } else {
            const tOut    = T - dMax;
            const tChange = bothDiffer ? T - dMin : tOut;
            const hasSpread = bothDiffer;
            // Show axis labels as negative values relative to the capture edge.
            const rel = (abs) => (abs >= T ? 0 : abs - T);
            // Boundary row (uncertainty-adjusted limits)
            if (su > 0) {
                const abs = Math.max(0, tOut - su);
                addTick(abs, `${rel(abs).toPrecision(3)}`, 0);
            }
            if (hu > 0) {
                const abs = Math.min(T, tChange + hu);
                addTick(abs, `${rel(abs).toPrecision(3)}`, 0);
            }
            // Nominal row
            if (showBoth) {
                addTick(T - riseSet.max, `${(-riseSet.max).toPrecision(3)}`, 1);
                addTick(T - fallSet.max, `${(-fallSet.max).toPrecision(3)}`, 1);
                if (riseSet.hasSpread) addTick(T - riseSet.min, `${(-riseSet.min).toPrecision(3)}`, 0);
                if (fallSet.hasSpread) addTick(T - fallSet.min, `${(-fallSet.min).toPrecision(3)}`, 0);
            } else {
                addTick(tOut,    `${(-dMax).toPrecision(3)}`, 1);
                if (hasSpread) addTick(tChange, `${(-dMin).toPrecision(3)}`, 1);
            }
        }

        // Render: tick line + label in assigned row. Clamp label x to stay within wave area.
        for (const { x, label, row } of pendingTicks) {
            this._svgLine(svg, x, AXIS_Y - 2, x, AXIS_Y + 3, 'var(--canvas-axis)', 1);
            const ty   = TICK_ROW[row];
            const half = label.length * 2.5;
            const lx2  = Math.max(LABEL_W + half + 1, Math.min(LABEL_W + WAVE_W - half - 1, x));
            this._svgText(svg, lx2, ty, label, 'var(--canvas-label)', 8, 'middle');
        }

        // ── Legend ────────────────────────────────────────────────────────────
        // Rendered left-to-right in the wave area so it cannot overflow either edge.
        // Falls back to short labels if full labels don't fit.
        // Every item gets an SVG <title> tooltip so hovering explains what the
        // swatch means (critical for the uncertainty bands that used to show
        // the opaque "Su"/"Hu" abbreviations).
        const hasSpreadLegend = bothDiffer;
        const suStr = entry.uncertainty_setup != null ? entry.uncertainty_setup.toPrecision(3) : '';
        const huStr = entry.uncertainty_hold  != null ? entry.uncertainty_hold.toPrecision(3)  : '';
        const COL_MARKER = 'var(--sdc-wf-marker-fill)';
        const COL_FALL   = 'var(--sdc-wf-fall-fill)';
        const suFull = `${TIMING_LABELS.setup_unc.short}=${suStr}${timeUnit}`;
        const huFull = `${TIMING_LABELS.hold_unc.short}=${huStr}${timeUnit}`;
        // Legend arrival entries — match the marker visual:
        //   • showBoth (rise ≠ fall) or both equal at a point → two entries
        //     (yellow rise / pink fall). The split-color marker is half each.
        //   • amber spread band (single transition with min/max range, OR
        //     both transitions equal with a range) → single ↑/↓/↑↓ entry.
        //   • single point, only rise → one yellow entry; only fall → one pink.
        const W = entry.is_input ? 'arrival' : 'output';
        const tipRise = `rising-edge ${W} — set with -rise`;
        const tipFall = `falling-edge ${W} — set with -fall`;
        const riseFallEntries = [
            { color: COL_MARKER, full: '↑ rise',  short: '↑', tip: tipRise },
            { color: COL_FALL,   full: '↓ fall',  short: '↓', tip: tipFall },
        ];
        let arrivalEntries;
        if (showBoth) {
            arrivalEntries = riseFallEntries;
        } else if (hasSpreadLegend) {
            // Amber spread band (riseSet==fallSet with min/max range, or only
            // one transition has a range). Symbol indicates which edges are
            // constrained.
            const sym = (riseSet && fallSet) ? '↑↓'
                      : riseSet ? '↑' : '↓';
            const tip = entry.is_input
                ? `${W} window — [dMin, dMax] from set_input_delay`
                : `${W} window — [T−dMax, T−dMin] from set_output_delay`;
            arrivalEntries = [{
                color: COL_UNCERT,
                full: `${sym} ${W}`, short: sym, tip,
            }];
        } else if (riseSet && fallSet) {
            // Both equal at a point — split-color marker, matched legend.
            arrivalEntries = riseFallEntries;
        } else if (riseSet) {
            arrivalEntries = [{
                color: COL_MARKER, full: `↑ ${W}`, short: '↑', tip: tipRise,
            }];
        } else {
            arrivalEntries = [{
                color: COL_FALL,   full: `↓ ${W}`, short: '↓', tip: tipFall,
            }];
        }
        const legendDef = entry.is_input ? [
            { color: COL_INVALID,  full: 'invalid', short: 'inv',
              tip: 'data not yet valid — before the allowed arrival window' },
            ...(hu > 0 ? [{ color: COL_HOLD_UNC,  full: huFull, short: 'hld unc',
                           tip: TIMING_LABELS.hold_unc.tip }] : []),
            ...arrivalEntries,
            ...(su > 0 ? [{ color: COL_SETUP_UNC, full: suFull, short: 'set unc',
                           tip: TIMING_LABELS.setup_unc.tip }] : []),
            { color: COL_VALID,    full: 'valid',   short: 'ok',
              tip: 'data definitely stable — safe for the capture edge' },
        ] : [
            { color: COL_VALID,    full: 'valid',   short: 'ok',
              tip: 'data definitely stable — output ready before the deadline' },
            ...(su > 0 ? [{ color: COL_SETUP_UNC, full: suFull, short: 'set unc',
                           tip: TIMING_LABELS.setup_unc.tip }] : []),
            ...arrivalEntries,
            ...(hu > 0 ? [{ color: COL_HOLD_UNC,  full: huFull, short: 'hld unc',
                           tip: TIMING_LABELS.hold_unc.tip }] : []),
            { color: COL_INVALID,  full: 'invalid', short: 'inv',
              tip: 'output may still be changing — past the allowed window' },
        ];
        const legItemW = (label) => label.length * 5.5 + 16;
        const totalFullW = legendDef.reduce((s, it) => s + legItemW(it.full), 0);
        const legend = legendDef.map(it => ({
            color: it.color,
            label: totalFullW <= WAVE_W ? it.full : it.short,
            tip: it.tip,
        }));
        let lx = LABEL_W + 4;
        for (const { color, label, tip } of legend) {
            const tw = legItemW(label);
            if (lx + tw > LABEL_W + WAVE_W - 4) break;
            const swatch = document.createElementNS(SVG_NS, 'rect');
            swatch.setAttribute('x', lx);
            swatch.setAttribute('y', LEG_Y - 7);
            swatch.setAttribute('width', 8);
            swatch.setAttribute('height', 8);
            swatch.style.fill = color;
            swatch.style.fillOpacity = BAR_OPACITY;
            this._svgTitle(swatch, tip);
            svg.appendChild(swatch);
            const textEl = this._svgText(svg, lx + 10, LEG_Y, label,
                                         'var(--canvas-label)', 8, 'start');
            this._svgTitle(textEl, tip);
            lx += tw;
        }

        return svg;
    }

    // Render one combined diagram for a group of port-delay entries that
    // share the same clock + period — typically several set_input_delay
    // constraints against different clock edges of the same clock (rise +
    // -clock_fall). The result is ONE clock waveform on top + ONE data bar
    // beneath, colored by the *state of the pin* over time (not the union):
    //
    //   For input ports (data driven onto the pin, captured by us):
    //     gray  : data has not yet arrived from any launch
    //     amber : data is currently arriving for some launch (with hold/setup uncertainty)
    //     green : data from the most-recent launch is stable on the pin
    //   ⇒ N launches produce: gray → amber → green → amber → green → … → green.
    //
    //   For output ports (we drive the pin, external sink captures):
    //     gray  : design is currently changing the output
    //     amber : output settling within a capture's stability window
    //     green : output is stable for capture
    //   ⇒ N captures produce: gray → amber → green → gray → amber → green → … → gray.
    //
    // Markers per entry stay color-coded (rise=yellow ↑, fall=pink ↓), placed
    // at the absolute-time positions (rise launch at t=0, fall launch at t=T/2).
    //
    // Falls back to the single-entry renderer when entries.length === 1.
    _renderPdMultiLane(entries, timeUnit) {
        if (!entries || entries.length === 0) return null;
        if (entries.length === 1) {
            return this._renderPortDelayDiagram(entries[0], timeUnit);
        }
        const T = entries[0].clk_period;
        if (!T || T <= 0) {
            // No clock period — fall back to per-entry text rows.
            const wrap = document.createElement('div');
            for (const e of entries) {
                wrap.appendChild(this._renderPortDelayFallback(e, timeUnit));
            }
            return wrap;
        }

        const N = entries.length;
        const LABEL_W = 200;
        const WAVE_W = Math.max(260, this._pdDiagramContainerW() - LABEL_W);

        // Layout — single bar like the per-entry diagram, but with N
        // annotation rows beneath (one per source constraint) so each
        // marker can be tied back to a numeric value.
        const { LEG_Y, CLK_HIGH, CLK_LOW, CLK_MID } = DIAGRAM_CONST;
        const DATA_TOP = 52;
        const DATA_BOT = 72;
        const ANNOT_LH = 11;
        const AXIS_Y   = DATA_BOT + 10 + N * ANNOT_LH;
        const TICK_ROW = [AXIS_Y + 11, AXIS_Y + 23];
        const SVG_H    = AXIS_Y + 34;

        const PRE   = T * 0.15;
        const POST  = T * 0.08;
        const TOTAL = T + PRE + POST;
        const txFull = (t) => LABEL_W + ((t + PRE) / TOTAL) * WAVE_W;

        const svg = this._makeSvg(LABEL_W + WAVE_W, SVG_H);

        // ── Clock waveform (drawn once) ───────────────────────────────────────
        // Always start with a rising edge at t=0 — the most natural baseline
        // when we have to display BOTH rise and fall launching edges.
        const edges = [
            { t: 0,     isRise: true  },
            { t: T / 2, isRise: false },
            { t: T,     isRise: true  },
        ];
        const clkPath = this._drawClockPath(
            svg, edges, txFull, CLK_HIGH, CLK_LOW, CLK_LOW,
            -PRE, T + POST, 'var(--canvas-axis)', '1.5');
        if (clkPath) {
            const clkName = entries[0] && entries[0].clock;
            this._svgTitle(clkPath,
                `${clkName || '(virtual)'} — period ${T.toPrecision(3)}`
                + `${timeUnit}. Multi-lane: ${entries.length} entries `
                + 'collapsed onto one shared waveform.');
        }

        // Guide lines — color-coded to the event they mark so the user can
        // see at a glance which clock edge launched/captured what:
        //   rise launch (t=0)   → magenta (rise marker color)
        //   fall launch (t=T/2) → cyan    (fall marker color), only if used
        //   capture (t=T)       → solid axis color
        // Using the marker color reinforces the visual association between
        // the guide line and the matching arrival/output marker on the bar.
        const COL_RISE = 'var(--sdc-wf-marker-fill)';
        const COL_FALL_GUIDE = 'var(--sdc-wf-fall-fill)';
        const guideRise = entries.some(e =>
            (e.is_input  && (e.clk_edge !== 'fall')) ||
            (!e.is_input && (e.clk_edge !== 'fall')));
        const guideFall = entries.some(e =>
            (e.is_input  && e.clk_edge === 'fall') ||
            (!e.is_input && e.clk_edge === 'fall'));
        const drawGuide = (t, color, dash, label) => {
            const rx = txFull(t);
            const line = this._svgLine(svg, rx, CLK_HIGH, rx, AXIS_Y,
                                       color, '1');
            if (dash) line.setAttribute('stroke-dasharray', dash);
            line.setAttribute('opacity', '0.6');
            if (label) this._svgTitle(line, label);
        };
        if (guideRise) drawGuide(0,     COL_RISE,            '4,3',
            'rising-edge launch — t=0 of the period; marks where '
            + '-rise (or unflagged) input/output_delay constraints anchor.');
        if (guideFall) drawGuide(T / 2, COL_FALL_GUIDE,      '4,3',
            'falling-edge launch — mid-period; marks where -fall '
            + 'input/output_delay constraints anchor.');
        // Capture edge: depends on whether any output uses fall-capture.
        const captureAtHalf = entries.some(e => !e.is_input && e.clk_edge === 'fall');
        const captureAtFull = entries.some(e =>  e.is_input  || e.clk_edge !== 'fall');
        if (captureAtFull) drawGuide(T, 'var(--canvas-axis)', null,
            `capture edge — t=T (${T.toPrecision(3)}${timeUnit}); the `
            + 'downstream flop samples on this edge.');
        if (captureAtHalf) drawGuide(T / 2, 'var(--canvas-axis)', null,
            'capture edge — mid-period (-clock_fall); the downstream '
            + 'flop samples on the falling edge.');

        this._svgText(svg, 4, CLK_MID + 4, 'CLK', 'var(--fg-muted)', 9, 'start');

        // ── Single shared data bar — overlay markers from every entry ───────
        // Band fills are routed through BAND_COLORS (see _svgBandRect); the
        // local COL_* aliases are kept for the legend code below, which
        // renders swatches by their CSS color value rather than by kind.
        const COL_INVALID   = BAND_COLORS['invalid'];
        const COL_UNCERT    = BAND_COLORS['uncert'];
        const COL_VALID     = BAND_COLORS['valid'];
        const COL_SETUP_UNC = BAND_COLORS['setup-unc'];
        const COL_HOLD_UNC  = BAND_COLORS['hold-unc'];
        const COL_MARKER    = 'var(--sdc-wf-marker-fill)';
        const COL_FALL      = 'var(--sdc-wf-fall-fill)';
        const { BAR_OPACITY } = DIAGRAM_CONST;

        const makeSet = (mx, mn) => this._makeDelaySet(mx, mn);
        // Hover descriptions match the legend strip; same lookup used
        // by the single-lane diagram. Multi-lane mixes input + output
        // entries only when every entry shares a direction (the
        // isInputAll / isOutputAll guards below), so we can pick the
        // right side of BAND_TIPS up-front.
        const tipForKind = entries.length && entries.every(e => e.is_input)
            ? BAND_TIPS.input
            : BAND_TIPS.output;
        const drawRect = (x1, x2, kind, opacity = BAR_OPACITY) => {
            const r = this._svgBandRect(svg, x1, DATA_TOP, x2 - x1,
                                        DATA_BOT - DATA_TOP, kind, opacity);
            if (r && tipForKind[kind]) this._svgTitle(r, tipForKind[kind]);
            return r;
        };

        // Pre-compute each entry's transit-window endpoints in absolute time.
        // Bands may extend past 0 or T (when uncertainty is large) — those
        // wrap to the other end of the period since the clock is cyclic.
        //
        // For input:
        //   holdStart = t_launch + dMin - hu   data may already be changing
        //   holdEnd   = t_launch + dMin        earliest stable arrival
        //   spreadEnd = t_launch + dMax        latest stable arrival
        //   setupEnd  = t_launch + dMax + su   data definitely settled
        // For output (mirrored around the capture edge):
        //   setupStart = t_capture - dMax - su  output must be stable from here
        //   setupEnd   = t_capture - dMax       output stability deadline
        //   spreadEnd  = t_capture - dMin       latest possible transition end
        //   holdEnd    = t_capture - dMin + hu  output may have started changing
        const computed = entries.map(entry => {
            const su = entry.uncertainty_setup || 0;
            const hu = entry.uncertainty_hold  || 0;
            const riseSet = makeSet(entry.rise_max, entry.rise_min);
            const fallSet = makeSet(entry.fall_max, entry.fall_min);
            const tLaunch  = entry.is_input
                ? (entry.clk_edge === 'fall' ? T / 2 : 0) : 0;
            const tCapture = entry.is_input
                ? T : (entry.clk_edge === 'fall' ? T / 2 : T);
            const sets = [riseSet, fallSet].filter(s => s);
            const dMax = sets.length ? Math.max(...sets.map(s => s.max)) : 0;
            const dMin = sets.length ? Math.min(...sets.map(s => s.min)) : 0;
            // Band endpoints in absolute time (UN-clamped — wrap handler
            // splits them into in-range and wrapped portions when needed).
            const holdStart  = entry.is_input ? (tLaunch + dMin - hu)
                                              : (tCapture - dMin + hu);
            const holdEnd    = entry.is_input ? (tLaunch + dMin)
                                              : (tCapture - dMin);
            const spreadEnd  = entry.is_input ? (tLaunch + dMax)
                                              : (tCapture - dMax);
            const setupEnd   = entry.is_input ? (tLaunch + dMax + su)
                                              : (tCapture - dMax - su);
            // For input the bands span [holdStart, setupEnd] left-to-right.
            // For output (mirrored): [setupEnd, holdStart], with setupEnd
            // smaller than holdStart in absolute time.
            const transitL = entry.is_input ? holdStart  : setupEnd;
            const transitR = entry.is_input ? setupEnd   : holdStart;
            return { entry, su, hu, riseSet, fallSet, tLaunch, tCapture,
                     dMax, dMin,
                     holdStart, holdEnd, spreadEnd, setupEnd,
                     transitL, transitR };
        }).filter(c => c.riseSet || c.fallSet);

        const isInputAll  = entries.every(e =>  e.is_input);
        const isOutputAll = entries.every(e => !e.is_input);
        const xLeft = txFull(0);
        const xRight = txFull(T);

        // Wrap-aware rectangle painter. A band may straddle 0 or T (clock
        // is cyclic with period T); we draw the in-range portion plus any
        // wrapped portion at the far end of the period.
        const drawWrapped = (t1, t2, kind, opacity = BAR_OPACITY) => {
            if (t2 <= t1) return;
            // In-range portion clamped to [0, T].
            const inA = Math.max(0, t1);
            const inB = Math.min(T, t2);
            if (inB > inA) drawRect(txFull(inA), txFull(inB), kind, opacity);
            // Wrapped portion from the negative side: [T + t1, T].
            if (t1 < 0) {
                const wA = Math.max(0, T + t1);
                const wB = Math.min(T, t2 < 0 ? T + t2 : T);
                if (wB > wA) drawRect(txFull(wA), txFull(wB), kind, opacity);
            }
            // Wrapped portion from the positive side: [0, t2 - T].
            if (t2 > T) {
                const wA = Math.max(0, t1 > T ? t1 - T : 0);
                const wB = Math.min(T, t2 - T);
                if (wB > wA) drawRect(txFull(wA), txFull(wB), kind, opacity);
            }
        };

        // Step 1 — base layer: paint default colour for the whole period.
        //   Input  : default GRAY (no data has arrived since t=0 in this cycle).
        //            Between transit windows we'll override with GREEN below.
        //   Output : default GREEN (output is stable from previous cycle's
        //            settled state). Between transit windows we'll override
        //            with GRAY below (output is changing for the next capture).
        if (isInputAll) {
            drawRect(xLeft, xRight, 'invalid');
        } else if (isOutputAll) {
            drawRect(xLeft, xRight, 'valid');
        } else {
            drawRect(xLeft, xRight, 'valid', 0.4);
        }

        // Step 2 — paint the "between transit" gaps.
        // Sort entries by transit window left-edge, then for each consecutive
        // pair paint the gap with the appropriate "between" colour.
        const sorted = [...computed].sort((a, b) => a.transitL - b.transitL);
        let anyHold = false, anySpread = false, anySetup = false;
        let anyGreen = false, anyGray = false;
        if (isInputAll) {
            // After each entry's setupEnd, until the NEXT entry's holdStart
            // (or end of period), data from THIS entry's launch is stable
            // → GREEN. Wraps cyclically: after the last entry, the green
            // continues to the first entry's holdStart (possibly wrapping
            // around through t=T → t=0).
            for (let i = 0; i < sorted.length; i++) {
                const cur = sorted[i];
                const nxt = sorted[(i + 1) % sorted.length];
                // Cyclic: distance from cur.setupEnd to nxt.holdStart, modulo T.
                let g0 = cur.setupEnd;
                let g1 = nxt.holdStart;
                if (g1 < g0) g1 += T;  // wrap
                if (g1 > g0) {
                    drawWrapped(g0, g1, 'valid');
                    anyGreen = true;
                }
            }
            // The "starting gray" survives wherever no entry's transit OR
            // green gap covers — i.e., before the FIRST entry's holdStart
            // when no green wraps in from the previous cycle. Since we
            // painted gray as the base layer, there's nothing more to do
            // for the gray itself; it shows through wherever bands and
            // green gaps don't overlay.
            anyGray = true;
        } else if (isOutputAll) {
            // For output: between captures the output is changing.
            // Paint GRAY between cur.holdEnd and nxt.setupStart (wrapping).
            for (let i = 0; i < sorted.length; i++) {
                const cur = sorted[i];
                const nxt = sorted[(i + 1) % sorted.length];
                let g0 = cur.holdStart;
                let g1 = nxt.setupEnd;
                if (g1 < g0) g1 += T;
                if (g1 > g0) {
                    drawWrapped(g0, g1, 'invalid');
                    anyGray = true;
                }
            }
            anyGreen = true;  // base layer is green
        }

        // Step 3 — overlay each entry's transit bands (hold_unc, spread, setup_unc).
        for (const c of sorted) {
            if (isInputAll) {
                // [holdStart, holdEnd]   → hold uncertainty (cyan)
                // [holdEnd,   spreadEnd] → arrival spread (amber, only if min!=max)
                // [spreadEnd, setupEnd]  → setup uncertainty (orange)
                if (c.hu > 0) {
                    drawWrapped(c.holdStart, c.holdEnd, 'hold-unc');
                    anyHold = true;
                }
                if (c.dMax > c.dMin) {
                    drawWrapped(c.holdEnd, c.spreadEnd, 'uncert');
                    anySpread = true;
                }
                if (c.su > 0) {
                    drawWrapped(c.spreadEnd, c.setupEnd, 'setup-unc');
                    anySetup = true;
                }
            } else if (isOutputAll) {
                // Mirrored: setup_unc, spread, hold_unc
                if (c.su > 0) {
                    drawWrapped(c.setupEnd, c.spreadEnd, 'setup-unc');
                    anySetup = true;
                }
                if (c.dMax > c.dMin) {
                    drawWrapped(c.spreadEnd, c.holdEnd, 'uncert');
                    anySpread = true;
                }
                if (c.hu > 0) {
                    drawWrapped(c.holdEnd, c.holdStart, 'hold-unc');
                    anyHold = true;
                }
            }
        }

        // Bar outline (single shared bar).
        this._svgOutlineRect(svg, xLeft, DATA_TOP,
                             xRight - xLeft, DATA_BOT - DATA_TOP);

        // ── Per-entry markers + annotation rows ───────────────────────────────
        const pendingTicks = [];
        const addTick = (t, label, row) => {
            const x = Math.max(LABEL_W + 1, Math.min(LABEL_W + WAVE_W - 1, txFull(t)));
            pendingTicks.push({ x, label, row });
        };
        addTick(0, '0', 1);
        addTick(T, `T=${T.toPrecision(3)}${timeUnit}`, 1);

        // CLK row label is shared.
        this._svgText(svg, 4, DATA_TOP + 9,
                      entries[0].clock || '(any)', 'var(--canvas-label)', 9, 'start');

        // Legend: only the categories actually painted on the bar.
        const legend = {
            invalid:    anyGray,
            valid:      anyGreen,
            holdUnc:    anyHold,
            uncertBand: anySpread,
            setupUnc:   anySetup,
            riseMarker: false, fallMarker: false,
        };

        for (let i = 0; i < computed.length; i++) {
            const c = computed[i];
            const { entry, riseSet, fallSet, tLaunch, tCapture, dMax, dMin, su, hu } = c;
            const allMax = [riseSet, fallSet].filter(s => s).map(s => s.max);
            const allMin = [riseSet, fallSet].filter(s => s).map(s => s.min);
            const showBoth = riseSet && fallSet
                && (riseSet.min !== fallSet.min || riseSet.max !== fallSet.max);
            const hasSpread = dMin !== dMax;

            // Marker symbol + color is determined by the entry's *launching*
            // edge of the clock (set_input_delay rise vs -clock_fall), NOT by
            // which data transition (rise_max vs fall_max) is set. Each entry
            // gets ONE marker so the user can match it to one constraint.
            const launchSym = entry.clk_edge === 'fall' ? '↓' : '↑';
            const launchCol = entry.clk_edge === 'fall' ? COL_FALL : COL_MARKER;
            if (entry.clk_edge === 'fall') legend.fallMarker = true;
            else                            legend.riseMarker = true;

            // Marker x-position — for input the arrival point (tLaunch + dMax),
            // for output the stability point (tCapture - dMax).
            const xPoint = entry.is_input
                ? txFull(tLaunch + dMax)
                : txFull(tCapture - dMax);
            // Single colored line + arrow symbol. No black outline: the amber
            // band already provides high-contrast backing for the marker, and
            // the outline would otherwise read as a black bar at the end of
            // each arrival window. No spread bracket either — the amber band
            // already conveys the [dMin, dMax] arrival window.
            const marker = this._svgLine(svg, xPoint, DATA_TOP, xPoint,
                                         DATA_BOT, launchCol, 3);
            marker.dataset.marker = entry.clk_edge === 'fall' ? 'fall' : 'rise';
            marker.dataset.timeMax = String(dMax);
            this._svgText(svg, xPoint, DATA_TOP - 2, launchSym, launchCol, 9, 'middle');
            // If the entry has rise!=fall (rare with -add_delay), draw the
            // "other" transition as a second thinner tick to disambiguate.
            if (showBoth) {
                const xOther = entry.is_input
                    ? txFull(tLaunch + Math.min(...allMax))
                    : txFull(tCapture - Math.min(...allMax));
                if (Math.abs(xOther - xPoint) > 1) {
                    this._svgLine(svg, xOther, DATA_TOP + 4, xOther, DATA_BOT - 4,
                                  launchCol, 1.5);
                }
            }
            void hasSpread;

            // Per-entry annotation row beneath the bar.
            const launchTxt = entry.is_input
                ? `${launchSym} launch`
                : `${launchSym} capture`;
            const fmtNum = (v) => (entry.is_input ? v : -v).toPrecision(3);
            let valTxt;
            if (showBoth) {
                const fmtSet = (s, sym) => s.hasSpread
                    ? `${sym}${fmtNum(s.min)}–${fmtNum(s.max)}`
                    : `${sym}${fmtNum(s.max)}`;
                valTxt = `${fmtSet(riseSet, '↑')} / ${fmtSet(fallSet, '↓')}${timeUnit}`;
            } else if (hasSpread) {
                valTxt = `${fmtNum(dMin)}–${fmtNum(dMax)}${timeUnit}`;
            } else {
                valTxt = `${fmtNum(dMax)}${timeUnit}`;
            }
            const annot = `${launchTxt}  ${valTxt}` +
                          (su > 0 || hu > 0
                            ? `  (su=${su.toPrecision(3)} hu=${hu.toPrecision(3)})`
                            : '');
            this._svgText(svg, 4, DATA_BOT + 10 + i * ANNOT_LH, annot,
                          'var(--canvas-label)', 8, 'start');

            // Per-entry ticks — labels show ABSOLUTE time on the period
            // axis, not the launch-relative constraint value (which is
            // already shown in the per-entry annotation row above the
            // axis). Otherwise the fall-launched lane's tick reads "0.2"
            // even though it lands at T/2 + 0.2, making the axis look
            // like it restarted at zero.
            //
            // Wrap-aware: if a tick lands outside [0, T] (e.g. the hold
            // boundary precedes the first rising edge), also place the
            // tick at the wrapped position so it tracks the visible band.
            const absTick = (tAbs, row = 1) => {
                const inRange = tAbs >= 0 && tAbs <= T;
                const wrapped = tAbs < 0 ? tAbs + T : tAbs > T ? tAbs - T : null;
                if (inRange) addTick(tAbs, `${tAbs.toPrecision(3)}`, row);
                if (wrapped != null && wrapped >= 0 && wrapped <= T) {
                    addTick(wrapped, `${tAbs.toPrecision(3)}`, row);
                }
            };
            // Nominal arrival/output points (row 1 = bottom).
            if (showBoth) {
                absTick(entry.is_input ? tLaunch + riseSet.max : tCapture - riseSet.max);
                absTick(entry.is_input ? tLaunch + fallSet.max : tCapture - fallSet.max);
            } else {
                absTick(entry.is_input ? tLaunch + dMax : tCapture - dMax);
                if (hasSpread) absTick(entry.is_input ? tLaunch + dMin : tCapture - dMin);
            }
            // Boundary ticks for setup / hold uncertainty (row 0 = top).
            // Same convention as the single-entry diagram: ticks identify
            // the edge of each uncertainty band.
            if (hu > 0) {
                absTick(entry.is_input
                    ? (tLaunch  + dMin - hu)        // input: hold band starts here
                    : (tCapture - dMin + hu),       // output: hold band ends here
                    0);
            }
            if (su > 0) {
                absTick(entry.is_input
                    ? (tLaunch  + dMax + su)        // input: setup band ends here
                    : (tCapture - dMax - su),       // output: setup band starts here
                    0);
            }
        }

        // ── Shared axis ───────────────────────────────────────────────────────
        this._svgLine(svg, txFull(0), AXIS_Y, txFull(T), AXIS_Y, 'var(--canvas-axis)', 1);
        for (const { x, label, row } of pendingTicks) {
            this._svgLine(svg, x, AXIS_Y - 2, x, AXIS_Y + 3, 'var(--canvas-axis)', 1);
            const ty   = TICK_ROW[row];
            const half = label.length * 2.5;
            const lx2  = Math.max(LABEL_W + half + 1, Math.min(LABEL_W + WAVE_W - half - 1, x));
            this._svgText(svg, lx2, ty, label, 'var(--canvas-label)', 8, 'middle');
        }

        // ── Shared legend ────────────────────────────────────────────────────
        const isInputLeg = entries.every(e => e.is_input);
        const suStr = entries[0].uncertainty_setup != null
            ? entries[0].uncertainty_setup.toPrecision(3) : '';
        const huStr = entries[0].uncertainty_hold != null
            ? entries[0].uncertainty_hold.toPrecision(3)  : '';
        const legendDef = [];
        if (legend.invalid)
            legendDef.push({ color: COL_INVALID, full: 'invalid', short: 'inv',
                             tip: isInputLeg
                                ? 'data has not yet arrived from any launch'
                                : 'output is currently changing — not yet stable' });
        if (legend.holdUnc)
            legendDef.push({ color: COL_HOLD_UNC,
                             full: `${TIMING_LABELS.hold_unc.short}=${huStr}${timeUnit}`,
                             short: 'hld unc',
                             tip: TIMING_LABELS.hold_unc.tip });
        if (legend.uncertBand)
            legendDef.push({ color: COL_UNCERT,
                             full: isInputLeg ? 'arr window' : 'transition',
                             short: isInputLeg ? 'arr' : 'tr',
                             tip: isInputLeg
                                ? 'arrival window — [dMin, dMax] from set_input_delay'
                                : 'output transition window — [T−dMax, T−dMin]' });
        if (legend.setupUnc)
            legendDef.push({ color: COL_SETUP_UNC,
                             full: `${TIMING_LABELS.setup_unc.short}=${suStr}${timeUnit}`,
                             short: 'set unc',
                             tip: TIMING_LABELS.setup_unc.tip });
        if (legend.valid)
            legendDef.push({ color: COL_VALID, full: 'valid', short: 'ok',
                             tip: isInputLeg
                                ? 'data from the most recent launch is stable on the pin'
                                : 'output is stable for capture' });
        if (legend.riseMarker)
            legendDef.push({ color: COL_MARKER, full: '↑ rise', short: '↑',
                             tip: 'rising-edge launching/capturing constraint' });
        if (legend.fallMarker)
            legendDef.push({ color: COL_FALL,   full: '↓ fall', short: '↓',
                             tip: 'falling-edge launching/capturing constraint' });
        const legItemW = (label) => label.length * 5.5 + 16;
        const totalFullW = legendDef.reduce((s, it) => s + legItemW(it.full), 0);
        const legendItems = legendDef.map(it => ({
            color: it.color,
            label: totalFullW <= WAVE_W ? it.full : it.short,
            tip: it.tip,
        }));
        let lx = LABEL_W + 4;
        for (const { color, label, tip } of legendItems) {
            const tw = legItemW(label);
            if (lx + tw > LABEL_W + WAVE_W - 4) break;
            const swatch = document.createElementNS(SVG_NS, 'rect');
            swatch.setAttribute('x', lx);
            swatch.setAttribute('y', LEG_Y - 7);
            swatch.setAttribute('width', 8);
            swatch.setAttribute('height', 8);
            swatch.style.fill = color;
            swatch.style.fillOpacity = BAR_OPACITY;
            this._svgTitle(swatch, tip);
            svg.appendChild(swatch);
            const textEl = this._svgText(svg, lx + 10, LEG_Y, label,
                                         'var(--canvas-label)', 8, 'start');
            this._svgTitle(textEl, tip);
            lx += tw;
        }

        return svg;
    }

    // Draw a single arrival/output point marker at one x position. The visual
    // varies with which transitions are constrained, so the user can tell at
    // a glance whether they're looking at a rise-only, fall-only, or both
    // (rise == fall) constraint:
    //   • rise only  → solid yellow line + ↑
    //   • fall only  → solid pink   line + ↓
    //   • both equal → top half yellow, bottom half pink, label ↑↓
    _drawPdSinglePoint(svg, x, top, bot, riseSet, fallSet) {
        const RISE_COL = 'var(--sdc-wf-marker-fill)';
        const FALL_COL = 'var(--sdc-wf-fall-fill)';
        // Outline (always) — keeps the marker visible on any background.
        this._svgLine(svg, x, top, x, bot, 'var(--sdc-wf-marker-line)', 4);
        const kind = riseSet && fallSet ? 'both' : riseSet ? 'rise' : 'fall';
        // The "primary" line carries the data-marker attribute so tests can
        // count markers by transition kind without relying on color strings.
        // Time values come from whichever set drives the marker position.
        const timeMax = (kind === 'fall' ? fallSet : riseSet).max;
        const tagMarker = (line) => {
            line.dataset.marker = kind;
            line.dataset.timeMax = String(timeMax);
        };
        // One tooltip describing all constraint edges at this point.
        // Single-point means rise/fall sets are equal-or-one-sided,
        // so this short form covers every case.
        const tip = kind === 'both'
            ? `arrival/output point — both rise and fall constrain this `
              + `time (${timeMax})`
            : kind === 'rise'
              ? 'rise transition — set with -rise on the input/output_delay '
                + 'command (or both edges when no -rise/-fall flag)'
              : 'fall transition — set with -fall on the input/output_delay '
                + 'command';
        if (riseSet && fallSet) {
            const mid = (top + bot) / 2;
            const a = this._svgLine(svg, x, top, x, mid, RISE_COL, 2);
            tagMarker(a); this._svgTitle(a, tip);
            this._svgTitle(this._svgLine(svg, x, mid, x, bot, FALL_COL, 2), tip);
            this._svgText(svg, x, top - 2, '↑↓', RISE_COL, 9, 'middle');
        } else if (riseSet) {
            const a = this._svgLine(svg, x, top, x, bot, RISE_COL, 2);
            tagMarker(a); this._svgTitle(a, tip);
            this._svgText(svg, x, top - 2, '↑', RISE_COL, 9, 'middle');
        } else {
            const a = this._svgLine(svg, x, top, x, bot, FALL_COL, 2);
            tagMarker(a); this._svgTitle(a, tip);
            this._svgText(svg, x, top - 2, '↓', FALL_COL, 9, 'middle');
        }
    }

    // Draw a per-edge arrival marker on the data bar.
    //   set      = { min, max, hasSpread } in plotting coordinates
    //   fillCol  = CSS color for the marker (rise or fall)
    //   sym      = '↑' or '↓' label drawn just above the bar
    // Renders:
    //   - a thin black outline behind a colored vertical line at set.max
    //   - a left-cap line at set.min when hasSpread (so the range reads as a
    //     bracket, with the colored "main" line on the right)
    //   - the edge symbol above the marker for at-a-glance disambiguation
    _drawPdEdgeMarker(svg, txFull, set, top, bot, fillCol, sym) {
        const xMax = txFull(set.max);
        // Outline + colored fill so the marker reads on any background.
        this._svgLine(svg, xMax, top, xMax, bot, 'var(--sdc-wf-marker-line)', 4);
        // The colored fill line carries data-marker for test queries:
        //   data-marker="rise"|"fall" — transition kind driven by sym
        //   data-time-max             — the constrained max value
        //   data-time-min             — only when the constraint has a spread
        const kind = sym === '↑' ? 'rise' : 'fall';
        const main = this._svgLine(svg, xMax, top, xMax, bot, fillCol, 2);
        main.dataset.marker = kind;
        main.dataset.timeMax = String(set.max);
        // Hover tooltip describing what this marker carries. Three
        // possible shapes: max-only (a single arrival point at set.max),
        // min/max range (the bracket marker pair), or one-edge (only
        // rise OR only fall constrained).
        const role = kind === 'rise' ? 'rising-edge' : 'falling-edge';
        const maxTip = set.hasSpread
            ? `${role} latest constraint — max = ${set.max}`
            : `${role} arrival/output — set ${set.max} (set with -${kind})`;
        this._svgTitle(main, maxTip);
        if (set.hasSpread) {
            main.dataset.timeMin = String(set.min);
            const xMin = txFull(set.min);
            this._svgLine(svg, xMin, top, xMin, bot, 'var(--sdc-wf-marker-line)', 3);
            const minLine = this._svgLine(svg, xMin, top, xMin, bot, fillCol, 1.5);
            this._svgTitle(minLine,
                `${role} earliest constraint — min = ${set.min}`);
            this._svgLine(svg, xMin, top + 1, xMax, top + 1, fillCol, 1.5);
        }
        this._svgText(svg, xMax, top - 2, sym, fillCol, 9, 'middle');
    }

    // ── Clock Groups panel ────────────────────────────────────────────────────

    _buildClockGroupsPanel(container) {
        container.style.cssText = 'display:flex;flex-direction:column;height:100%;overflow:hidden;';

        const toolbar = document.createElement('div');
        toolbar.style.cssText =
            'display:flex;align-items:center;padding:2px 6px;gap:6px;border-bottom:1px solid var(--border);' +
            'background:var(--bg-header);flex-shrink:0;';
        const refreshBtn = document.createElement('button');
        refreshBtn.textContent = '↺ Refresh';
        refreshBtn.title = 'Reload clock group constraints from the current design';
        refreshBtn.style.cssText =
            'padding:2px 8px;font-size:12px;cursor:pointer;background:var(--bg-input);' +
            'color:var(--fg-primary);border:1px solid var(--border);border-radius:3px;';
        refreshBtn.addEventListener('click', () => {
            this._cgLoaded = false;
            this._cgLoading = false;
            this._loadModes();
            this._loadClockGroups();
        });
        toolbar.appendChild(refreshBtn);
        container.appendChild(toolbar);

        const scroll = document.createElement('div');
        scroll.style.cssText = 'flex:1;overflow-y:auto;padding:8px;background:var(--bg-main);';
        this._cgScrollArea = scroll;
        container.appendChild(scroll);
        scroll.innerHTML =
            '<div style="padding:12px;color:var(--fg-muted);font-style:italic;">Loading…</div>';
    }

    async _loadClockGroups() {
        if (this._cgLoaded || this._cgLoading) return;
        this._cgLoading = true;
        this._cgScrollArea.innerHTML =
            '<div style="padding:12px;color:var(--fg-muted);font-style:italic;">Loading…</div>';
        try {
            await this._app.websocketManager.readyPromise;
            // The relationship matrix builds its row/column set from
            // `this._clocks`, populated by sdc_clocks (loaded by _loadData).
            // If the user opens this tab first, sdc_clock_groups can come
            // back BEFORE sdc_clocks, leaving the matrix with an empty clock
            // list. Wait for the Clocks-tab data to land first so the matrix
            // always sees the full clock set.
            if (!this._loaded && !this._loading) {
                this._loadData();  // kick it off (no-op if already running)
            }
            // Brief await: at least one microtask so an in-flight _loadData
            // has a chance to settle before we render. _renderClockGroups
            // is also re-invoked from _loadData's success path (see below)
            // as a belt-and-suspenders measure for slow sdc_clocks responses.
            const data = await this._requestWithTimeout({ type: 'sdc_clock_groups' });
            this._cgData = data;  // remember so we can re-render later
            this._renderClockGroups(data);
            this._cgLoaded = true;
        } catch (e) {
            console.error('[SDC] clock groups load error:', e);
            this._showLoadError(this._cgScrollArea, 'clock groups', e, () => {
                this._cgLoaded = false;
                this._cgLoading = false;
                this._loadClockGroups();
            });
        } finally {
            this._cgLoading = false;
        }
    }

    _renderClockGroups(data) {
        this._cgScrollArea.innerHTML = '';
        // Backend emits {"groups":[...]} (matches the handleSdcClockGroups contract
        // and TestSdcHandler expectations). Earlier code read `clock_groups`, so
        // no groups ever appeared in the UI even when set_clock_groups was used.
        const groups = data.groups || [];

        const summary = document.createElement('div');
        summary.style.cssText =
            'padding:4px 8px;font-size:12px;color:var(--fg-muted);' +
            'border-bottom:1px solid var(--border-subtle);margin-bottom:8px;';
        summary.textContent = groups.length === 0
            ? 'No clock group constraints defined in SDC (no set_clock_groups).'
            : `${groups.length} clock group constraint${groups.length !== 1 ? 's' : ''}`;
        this._cgScrollArea.appendChild(summary);

        for (const grp of groups) {
            this._cgScrollArea.appendChild(this._makeClockGroupCard(grp));
        }

        // Relationship matrix.  We always emit every clock the design knows
        // about — no arbitrary "if more than N clocks, skip" cap.  Two
        // strategies, picked automatically by size:
        //
        //   • Small designs (≤ MATRIX_SINGLE) — one global matrix over
        //     every clock, exactly like the original design.  Every pair
        //     of clocks gets a cell, including pairs with no constraint
        //     (rendered as "·") so the user can scan for "what's missing".
        //
        //   • Large designs (> MATRIX_SINGLE) — split into per-command
        //     sub-matrices, one per `set_clock_groups` constraint.  Each
        //     sub-matrix only contains the clocks referenced by *that*
        //     command, so each one stays small + readable.  Plus a final
        //     listing of every clock not referenced by any command, since
        //     those have no constrained relationships to anyone.
        //
        // The threshold is heuristic, not a hard cap: above it the global
        // matrix is unwieldy to read (not unwieldy to render), so we offer
        // the per-command view as a more useful structure.
        const MATRIX_SINGLE = 32;
        const allClockNames = new Set();
        for (const grp of groups) {
            for (const set of (grp.clk_sets || [])) {
                for (const clk of set) allClockNames.add(clk);
            }
        }
        for (const clk of (this._clocks || [])) {
            if (clk && clk.name) allClockNames.add(clk.name);
        }

        // No set_clock_groups constraints at all → the matrix would be a
        // wall of "·" cells, which carries no useful information.  Show the
        // clock list as a flat scannable list instead so the user can still
        // see what clocks the design has.
        if (groups.length === 0 && allClockNames.size > 0) {
            this._cgScrollArea.appendChild(
                this._makeUnreferencedClocksList([...allClockNames].sort(), {
                    headerText:
                        `${allClockNames.size} clock${allClockNames.size === 1 ? '' : 's'} ` +
                        `(no set_clock_groups constraints — every pair is ` +
                        `treated as synchronous by default)`,
                    itemTitle: n => n,
                }));
        } else if (allClockNames.size >= 2 && allClockNames.size <= MATRIX_SINGLE) {
            this._cgScrollArea.appendChild(
                this._makeClockGroupMatrix(groups, [...allClockNames].sort()));
        } else if (allClockNames.size > MATRIX_SINGLE) {
            // Sub-matrix per command.  Stays small because each command
            // typically references only a handful of clocks.
            this._cgScrollArea.appendChild(
                this._makeMultiMatrixHeader(allClockNames.size, groups.length));
            const referenced = new Set();
            for (const grp of groups) {
                const grpClocks = new Set();
                for (const set of (grp.clk_sets || [])) {
                    for (const clk of set) {
                        grpClocks.add(clk);
                        referenced.add(clk);
                    }
                }
                if (grpClocks.size >= 2) {
                    this._cgScrollArea.appendChild(
                        this._makeClockGroupMatrix(
                            [grp], [...grpClocks].sort(),
                            { titleSuffix: grp.name ? ` — “${grp.name}”` : '' }));
                }
            }
            // Clocks that don't appear in any command — list them so the
            // user knows they exist (they have no constraint with anyone,
            // so they wouldn't be visible in any of the sub-matrices).
            const unreferenced = [...allClockNames].filter(
                n => !referenced.has(n)).sort();
            if (unreferenced.length > 0) {
                this._cgScrollArea.appendChild(
                    this._makeUnreferencedClocksList(unreferenced));
            }
        }
    }

    // Header explaining why the user is seeing many small matrices
    // instead of one big one.
    _makeMultiMatrixHeader(numClocks, numGroups) {
        const div = document.createElement('div');
        div.style.cssText =
            'margin-top:14px;padding:6px 10px;font-size:12px;color:var(--fg-muted);' +
            'background:var(--bg-panel);border:1px solid var(--border-subtle);' +
            'border-radius:4px;';
        div.innerHTML =
            `<div style="font-weight:600;color:var(--fg-primary);margin-bottom:2px;">` +
            `Per-command relationship matrices</div>` +
            `<div>This design has ${numClocks} clocks across ${numGroups} ` +
            `<code>set_clock_groups</code> command${numGroups === 1 ? '' : 's'}. ` +
            `Showing one matrix per command (each matrix lists only the clocks ` +
            `that command references) so the structure stays readable. ` +
            `Clocks that aren't referenced by any command are listed at the bottom.</div>`;
        return div;
    }

    _makeUnreferencedClocksList(names, opts = {}) {
        const section = document.createElement('div');
        section.style.cssText =
            'margin-top:12px;border:1px solid var(--border);border-radius:4px;' +
            'background:var(--bg-panel);';
        const hdr = document.createElement('div');
        hdr.style.cssText =
            'padding:5px 10px;background:var(--bg-header);font-size:12px;' +
            'font-weight:600;color:var(--fg-primary);' +
            'border-bottom:1px solid var(--border);';
        hdr.textContent = opts.headerText
            || `${names.length} clock${names.length === 1 ? '' : 's'} not referenced by any set_clock_groups command`;
        section.appendChild(hdr);
        // Vertical stacked list — one clock per row.  Use plain block layout
        // so rows stack with the natural line-box and don't depend on flex
        // sizing.  Each row carries an explicit line-height + small vertical
        // padding so adjacent rows can never visually overlap, regardless of
        // any inherited line-height.
        const body = document.createElement('div');
        body.style.cssText =
            'padding:6px 10px;font-family:monospace;font-size:12px;' +
            'color:var(--fg-primary);' +
            (names.length > 12 ? 'max-height:240px;overflow-y:auto;' : '');
        for (const n of names) {
            const row = document.createElement('div');
            row.style.cssText =
                'line-height:1.5;padding:1px 0;' + TRUNCATE_PATH_CSS;
            row.textContent = n;
            row.title = opts.itemTitle ? opts.itemTitle(n)
                                       : `${n} — no clock-groups constraint`;
            body.appendChild(row);
        }
        section.appendChild(body);
        return section;
    }

    _makeClockGroupCard(grp) {
        const card = document.createElement('div');
        card.style.cssText =
            'margin-bottom:8px;border:1px solid var(--border);border-radius:4px;overflow:hidden;font-size:12px;';

        const header = document.createElement('div');
        header.style.cssText =
            'display:flex;align-items:center;gap:8px;padding:5px 10px;' +
            'background:var(--bg-header);border-bottom:1px solid var(--border-subtle);';

        const typeColors = {
            asynchronous:
                'background:var(--sdc-async-bg);color:var(--sdc-async-fg);',
            logically_exclusive:
                'background:var(--sdc-lexcl-bg);color:var(--sdc-lexcl-fg);',
            physically_exclusive:
                'background:var(--sdc-pexcl-bg);color:var(--sdc-pexcl-fg);',
        };
        const badge = document.createElement('span');
        badge.style.cssText =
            `font-size:11px;padding:1px 6px;border-radius:3px;font-weight:600;white-space:nowrap;` +
            (typeColors[grp.type] || 'background:var(--bg-input);color:var(--fg-muted);');
        badge.textContent = grp.type === 'asynchronous'         ? 'ASYNC'
                          : grp.type === 'logically_exclusive'  ? 'LOGICALLY EXCL'
                          : grp.type === 'physically_exclusive' ? 'PHYSICALLY EXCL'
                          : (grp.type || 'UNKNOWN');
        header.appendChild(badge);

        if (grp.name) {
            const nameSpan = document.createElement('span');
            nameSpan.style.cssText = 'font-family:monospace;font-weight:600;color:var(--fg-primary);';
            nameSpan.textContent = grp.name;
            header.appendChild(nameSpan);
        }

        if (grp.allow_paths) {
            const allowSpan = document.createElement('span');
            allowSpan.style.cssText =
                'margin-left:auto;font-size:11px;color:var(--fg-muted);font-style:italic;';
            allowSpan.textContent = '-allow_paths';
            header.appendChild(allowSpan);
        }

        card.appendChild(header);

        const body = document.createElement('div');
        body.style.cssText = 'padding:6px 10px;display:flex;flex-direction:column;gap:4px;';

        const sets = grp.clk_sets || [];
        sets.forEach((set, i) => {
            const setRow = document.createElement('div');
            setRow.style.cssText = 'display:flex;align-items:center;gap:6px;flex-wrap:wrap;';

            const setLabel = document.createElement('span');
            setLabel.style.cssText = 'font-size:11px;color:var(--fg-muted);min-width:40px;flex-shrink:0;';
            setLabel.textContent = sets.length > 1 ? `Set ${i + 1}:` : 'Clocks:';
            setRow.appendChild(setLabel);

            for (const clk of set) {
                const chip = document.createElement('span');
                chip.style.cssText =
                    'font-family:monospace;font-size:12px;padding:1px 6px;border-radius:3px;' +
                    'background:var(--bg-input);color:var(--fg-primary);border:1px solid var(--border);';
                chip.textContent = clk;
                setRow.appendChild(chip);
            }
            body.appendChild(setRow);
        });

        if (body.children.length === 0) {
            const empty = document.createElement('span');
            empty.style.cssText = 'color:var(--fg-muted);font-style:italic;font-size:12px;';
            empty.textContent = '(no clock sets)';
            body.appendChild(empty);
        }

        card.appendChild(body);
        // SDC -comment from set_clock_groups.
        this._appendComment(card, grp.comment);
        return card;
    }

    // Compact relationship matrix showing pairwise clock relationships.
    // No size limit — when the global matrix would be unwieldy, the caller
    // (`_renderClockGroups`) splits the work into per-command sub-matrices,
    // each of which usually stays small.  Even at hundreds of clocks the
    // rendered table is fine: it sits inside an `overflow-x:auto` wrapper,
    // and the browser only paints the cells that scroll into view.
    //
    // Optional `opts.titleSuffix` appends extra text after "Clock
    // Relationship Matrix" in the header — used by the per-command view to
    // tag each sub-matrix with the originating constraint's name.
    _makeClockGroupMatrix(groups, clocks, opts = {}) {

        // Build (clockA, clockB sorted key) → {type, groupName}.
        // Includes both cross-set relationships (from the group's `type`) and
        // same-set relationships ('synchronous') so no cell is left ambiguous.
        //
        // Later rules overwrite earlier ones when the same pair is referenced
        // by more than one set_clock_groups command; cross-set constraints take
        // precedence over same-set since they are the "harder" constraint.
        const rel = new Map();
        const setKey = (a, b) => [a, b].sort().join('\x00');
        for (const grp of groups) {
            const sets = grp.clk_sets || [];
            // Same-set (synchronous) pairs first.
            for (const set of sets) {
                for (let i = 0; i < set.length; i++) {
                    for (let j = i + 1; j < set.length; j++) {
                        const k = setKey(set[i], set[j]);
                        if (!rel.has(k)) {
                            rel.set(k, { type: 'synchronous', group: grp.name });
                        }
                    }
                }
            }
            // Cross-set pairs override.
            for (let i = 0; i < sets.length; i++) {
                for (let j = i + 1; j < sets.length; j++) {
                    for (const a of sets[i]) {
                        for (const b of sets[j]) {
                            rel.set(setKey(a, b),
                                    { type: grp.type, group: grp.name });
                        }
                    }
                }
            }
        }

        const section = document.createElement('div');
        section.style.cssText =
            'margin-top:16px;border:1px solid var(--border);border-radius:4px;overflow:hidden;';

        const hdr = document.createElement('div');
        hdr.style.cssText =
            'padding:5px 10px;background:var(--bg-header);font-size:12px;font-weight:600;' +
            'color:var(--fg-primary);border-bottom:1px solid var(--border);';
        hdr.textContent = `Clock Relationship Matrix${opts.titleSuffix || ''}`;
        section.appendChild(hdr);

        const sub = document.createElement('div');
        sub.style.cssText =
            'padding:4px 10px;font-size:11px;color:var(--fg-muted);' +
            'background:var(--bg-panel);border-bottom:1px solid var(--border-subtle);';
        sub.textContent =
            'Each cell shows the timing relationship between the row and column clocks. ' +
            'Hover a cell to see which set_clock_groups constraint produced it.';
        section.appendChild(sub);

        const wrapper = document.createElement('div');
        wrapper.style.cssText = 'overflow-x:auto;padding:8px;';

        const table = document.createElement('table');
        table.style.cssText = 'border-collapse:collapse;font-size:11px;font-family:monospace;';

        // Column headers: rotated -45° so the full clock name is visible
        // without gobbling horizontal space. Standard pattern:
        //   - Each header cell has a fixed width that matches its data
        //     column below, so labels line up with their columns.
        //   - The rotated text is anchored at the column's center-bottom
        //     via `right: 50%` + `transform-origin: right bottom`, so it
        //     grows UP-LEFT into the empty space above earlier columns
        //     (rather than overlapping later columns to the right).
        // Data cells get the same CELL_W below so labels stay aligned.
        const CELL_W = 40;
        const longestName = clocks.reduce((m, c) => Math.max(m, c.length), 0);
        // A -45° tilted label of N chars (at 10px monospace ≈ 6px/char)
        // occupies roughly N * 6 * sin(45°) ≈ N * 4.2 pixels vertically.
        // Pad by a small constant; clamp so a very long name doesn't
        // consume the whole panel.
        const headerPx = Math.min(220, Math.max(60, longestName * 5 + 20));

        const thead = document.createElement('thead');
        const headerRow = document.createElement('tr');
        const emptyTh = document.createElement('th');
        emptyTh.style.cssText =
            `padding:3px 6px;height:${headerPx}px;vertical-align:bottom;`;
        headerRow.appendChild(emptyTh);
        for (const clk of clocks) {
            const th = document.createElement('th');
            th.style.cssText =
                `width:${CELL_W}px;min-width:${CELL_W}px;max-width:${CELL_W}px;` +
                `height:${headerPx}px;padding:0;position:relative;` +
                'border-bottom:1px solid var(--border);vertical-align:bottom;';
            // Two-step position so the rotation anchor is unambiguously the
            // column's center-bottom: first put the span's bottom-left edge
            // at the column center via left:50% + bottom:0 (no translate on
            // its own coords), then rotate about that left-bottom corner.
            // The label's baseline "grows" up and to the right from the
            // center of the column — each character sits directly above the
            // next column's left boundary, which visually reads as the label
            // pointing down at the correct column.
            const span = document.createElement('span');
            span.style.cssText =
                'position:absolute;left:50%;bottom:2px;' +
                'transform-origin:left bottom;' +
                'transform:rotate(-45deg);' +
                'white-space:nowrap;' +
                'font-family:monospace;font-size:11px;color:var(--fg-primary);';
            span.textContent = clk;
            span.title = clk;
            th.appendChild(span);
            headerRow.appendChild(th);
        }
        thead.appendChild(headerRow);
        table.appendChild(thead);

        // Color palette: the four kinds of cells need to be visually distinct.
        // Values live in style.css so they can follow the active light/dark
        // theme.  (They previously were hard-coded hexes that were hard to
        // read on the dark theme — too-dark-blue blended into the page bg.)
        //   sync   = gray (co-membership, not a constraint)
        //   async  = purple (no timing relationship)
        //   L.excl = green  (paths excluded by logic)
        //   P.excl = blue   (paths excluded by physics)
        const relStyles = {
            synchronous:          { bg: 'var(--sdc-sync-bg)',  fg: 'var(--sdc-sync-fg)',  abbr: 'sync'   },
            asynchronous:         { bg: 'var(--sdc-async-bg)', fg: 'var(--sdc-async-fg)', abbr: 'async'  },
            logically_exclusive:  { bg: 'var(--sdc-lexcl-bg)', fg: 'var(--sdc-lexcl-fg)', abbr: 'L.excl' },
            physically_exclusive: { bg: 'var(--sdc-pexcl-bg)', fg: 'var(--sdc-pexcl-fg)', abbr: 'P.excl' },
        };
        const relFullName = {
            // "sync" is emitted for clock pairs that appear inside the *same*
            // -group clause of a set_clock_groups command.  Per SDC semantics
            // those clocks are synchronous to each other regardless of the
            // command's -logically_exclusive / -physically_exclusive /
            // -asynchronous flag (the flag describes how clocks in *different*
            // -group clauses relate).  Spelling that out here so users don't
            // mistake a sync cell for a broken L.excl rendering.
            synchronous:
                'synchronous — listed in the same -group clause',
            asynchronous:         'asynchronous',
            logically_exclusive:  'logically exclusive',
            physically_exclusive: 'physically exclusive',
        };

        const tbody = document.createElement('tbody');
        clocks.forEach((rowClk) => {
            const tr = document.createElement('tr');

            const td0 = document.createElement('td');
            td0.style.cssText =
                'padding:3px 10px 3px 4px;color:var(--fg-primary);white-space:nowrap;' +
                'border-right:1px solid var(--border);font-weight:600;' +
                'text-align:right;font-family:monospace;';
            // Row labels show the full clock name — the table lives inside a
            // horizontally-scrollable wrapper, so long names can push the
            // data columns right without truncation.
            td0.textContent = rowClk;
            td0.title = rowClk;
            tr.appendChild(td0);

            clocks.forEach((colClk) => {
                const td = document.createElement('td');
                // Fixed column width matches the CELL_W used for the rotated
                // header cells so the labels line up with their data columns.
                td.style.cssText =
                    `width:${CELL_W}px;min-width:${CELL_W}px;max-width:${CELL_W}px;` +
                    'padding:3px 0;text-align:center;white-space:nowrap;' +
                    'border-bottom:1px solid var(--border-subtle);';

                if (rowClk === colClk) {
                    td.style.background = 'var(--bg-header)';
                    td.style.color = 'var(--fg-muted)';
                    td.textContent = '—';
                    td.title = `${rowClk} (diagonal — same clock, ` +
                        `no inter-clock relationship)`;
                } else {
                    const key = setKey(rowClk, colClk);
                    const info = rel.get(key);
                    const s = info ? relStyles[info.type] : null;
                    if (s) {
                        td.style.background = s.bg;
                        td.style.color = s.fg;
                        td.textContent = s.abbr;
                        const typeLabel = relFullName[info.type] || info.type;
                        td.title = info.group
                            ? `${rowClk} ↔ ${colClk}: ${typeLabel} (from "${info.group}")`
                            : `${rowClk} ↔ ${colClk}: ${typeLabel}`;
                    } else {
                        // No clock-groups command covers this pair — the
                        // "no group" cell users asked us to clarify.
                        td.style.color = 'var(--fg-muted)';
                        td.textContent = '·';
                        td.title =
                            `${rowClk} ↔ ${colClk}: no set_clock_groups ` +
                            `command covers this pair — they are treated ` +
                            `as synchronous (related) by default, so paths ` +
                            `are analyzed against the worst-case phase ` +
                            `alignment over the LCM of their periods. ` +
                            `Add set_clock_groups -asynchronous to mark ` +
                            `them unrelated and skip those paths.`;
                    }
                }
                tr.appendChild(td);
            });
            tbody.appendChild(tr);
        });
        table.appendChild(tbody);
        wrapper.appendChild(table);
        section.appendChild(wrapper);

        const legend = document.createElement('div');
        legend.style.cssText =
            'padding:4px 10px;font-size:11px;color:var(--fg-muted);display:flex;gap:14px;' +
            'flex-wrap:wrap;border-top:1px solid var(--border);';
        // Per-type expanded tooltip strings so a hover on the swatch
        // explains what a cell of that color means in plain English. The
        // matrix cells themselves carry pair-specific titles already; the
        // legend entries describe the category as a whole.
        const relLegendTip = {
            synchronous:
                'sync — clocks are listed in the same -group clause; ' +
                'paths between them are timed normally',
            asynchronous:
                'async — clocks are in different -group clauses of a ' +
                '-asynchronous set_clock_groups; cross-clock paths are ' +
                'not analyzed',
            logically_exclusive:
                'L.excl — different -group clauses of a ' +
                '-logically_exclusive set_clock_groups; the clocks are ' +
                'never simultaneously active in this design',
            physically_exclusive:
                'P.excl — different -group clauses of a ' +
                '-physically_exclusive set_clock_groups; the clocks ' +
                'physically cannot coexist (typically muxed clocks)',
        };
        for (const [type, { bg, abbr }] of Object.entries(relStyles)) {
            const item = document.createElement('span');
            item.style.cssText = 'display:flex;align-items:center;gap:3px;';
            const swatch = document.createElement('span');
            swatch.style.cssText =
                `display:inline-block;width:8px;height:8px;border-radius:2px;background:${bg};`;
            item.appendChild(swatch);
            item.appendChild(document.createTextNode(
                `${abbr} = ${(relFullName[type] || type).replace(/_/g, ' ')}`));
            this._setTooltip(item, relLegendTip[type] || relFullName[type]);
            legend.appendChild(item);
        }
        // Entry for pairs that no set_clock_groups command covers — this
        // is the "no group" cell users asked us to explain.
        const noneItem = document.createElement('span');
        noneItem.style.cssText = 'display:flex;align-items:center;gap:3px;';
        const noneSwatch = document.createElement('span');
        noneSwatch.style.cssText =
            'display:inline-block;width:8px;height:8px;border-radius:2px;' +
            'background:transparent;border:1px solid var(--border);';
        noneItem.appendChild(noneSwatch);
        noneItem.appendChild(document.createTextNode(
            '· = no clock-groups constraint'));
        this._setTooltip(noneItem,
            'no set_clock_groups command covers this pair — by default ' +
            'they are treated as synchronous (related), so STA analyzes ' +
            'paths between them against the worst-case phase alignment ' +
            'over the LCM of their periods. Add set_clock_groups ' +
            '-asynchronous to mark them unrelated and skip those paths.');
        legend.appendChild(noneItem);
        // Entry for the diagonal (clock paired with itself).
        const selfItem = document.createElement('span');
        selfItem.style.cssText = 'display:flex;align-items:center;gap:3px;';
        const selfSwatch = document.createElement('span');
        selfSwatch.style.cssText =
            'display:inline-block;width:8px;height:8px;border-radius:2px;' +
            'background:var(--bg-header);border:1px solid var(--border);';
        selfItem.appendChild(selfSwatch);
        selfItem.appendChild(document.createTextNode(
            '— = self (same clock)'));
        this._setTooltip(selfItem,
            'diagonal cells — a clock paired with itself; no inter-clock ' +
            'relationship applies');
        legend.appendChild(selfItem);
        section.appendChild(legend);

        return section;
    }

    // ── Endpoint panel ────────────────────────────────────────────────────────

    _buildEndpointPanel(container) {
        container.style.cssText = 'display:flex;flex-direction:column;height:100%;overflow:hidden;';

        // Toolbar row 1: "List endpoints" button + glob search box.
        // The "List endpoints" button mirrors the resolve-generated-clocks
        // UX: visible by default, triggers the (potentially expensive)
        // sta::Search::endpoints() walk on click, hides itself once the
        // analyzer cache is populated. Submitting a search with the input
        // ALSO triggers the populate (search needs the endpoint set).
        const toolbar = document.createElement('div');
        toolbar.style.cssText =
            'display:flex;align-items:center;padding:4px 8px;gap:6px;' +
            'border-bottom:1px solid var(--border);background:var(--bg-header);flex-shrink:0;';

        const listBtn = document.createElement('button');
        listBtn.textContent = '▶ List endpoints';
        listBtn.title =
            'Walk the timing analyzer\'s endpoint set and list every endpoint ' +
            '(flip-flops, latches, macro pins, combinational endpoints). Cached ' +
            'after the first call — this UI button only triggers the cache ' +
            'build, which on a million-instance design can take several seconds.';
        listBtn.style.cssText =
            'padding:3px 10px;font-size:12px;cursor:pointer;background:var(--accent-tab);' +
            'color:#fff;border:none;border-radius:3px;white-space:nowrap;';
        this._epListBtn = listBtn;
        listBtn.addEventListener('click',
            () => this._loadEndpointList({ reset: true }));
        toolbar.appendChild(listBtn);

        // Refresh button — only relevant after the first populate, since
        // before that there's nothing to refresh. Hidden by default; shown
        // alongside the kind filter once the list has been populated. The
        // design itself may change (re-link, new SDC, mode swap) — clicking
        // refresh re-fetches with the current pattern/kind filters and any
        // newly-tracked endpoints get picked up.
        const refreshBtn = document.createElement('button');
        refreshBtn.textContent = '↺ Refresh';
        refreshBtn.title =
            'Re-walk the endpoint set and reload the list. Use this after ' +
            'loading new SDC, re-linking the design, or running commands ' +
            'that change which pins are timing endpoints.';
        refreshBtn.style.cssText =
            'display:none;padding:3px 10px;font-size:12px;cursor:pointer;' +
            'background:var(--bg-input);color:var(--fg-primary);' +
            'border:1px solid var(--border);border-radius:3px;white-space:nowrap;';
        this._epRefreshBtn = refreshBtn;
        refreshBtn.addEventListener('click', () => {
            // Force a full re-fetch with the current filters. _epListAll is
            // cleared by the {reset:true} path inside _loadEndpointList.
            this._loadModes();
            this._loadEndpointList({ reset: true });
        });
        toolbar.appendChild(refreshBtn);

        const input = document.createElement('input');
        input.type = 'text';
        input.placeholder = 'Glob filter (e.g. u_top/* or reset) — Enter to search';
        input.style.cssText =
            'flex:1;padding:3px 8px;font-size:12px;font-family:monospace;' +
            'background:var(--bg-input);color:var(--fg-primary);border:1px solid var(--border);' +
            'border-radius:3px;outline:none;';
        this._epInput = input;
        input.addEventListener('keydown', e => {
            if (e.key === 'Enter') this._loadEndpointList({ reset: true });
        });
        const searchBtn = document.createElement('button');
        searchBtn.textContent = 'Search';
        searchBtn.style.cssText =
            'padding:3px 10px;font-size:12px;cursor:pointer;background:var(--bg-input);' +
            'color:var(--fg-primary);border:1px solid var(--border);border-radius:3px;' +
            'white-space:nowrap;';
        searchBtn.addEventListener('click',
            () => this._loadEndpointList({ reset: true }));
        toolbar.appendChild(input);
        toolbar.appendChild(searchBtn);
        container.appendChild(toolbar);

        // Toolbar row 2: instance filter buttons. The Endpoints tab now
        // surfaces every pin on the instance (inputs, outputs, CK), so
        // "instance filter" reads more naturally than "endpoint kind" —
        // the buttons gate which kind of *instance* (flop/latch/macro/
        // stdcell) gets a card, not which pin direction. Hidden until
        // the first list fetch lands so empty buttons aren't shown when
        // the user hasn't populated yet.
        const kindBar = document.createElement('div');
        kindBar.style.cssText =
            'display:none;align-items:center;padding:3px 8px;gap:6px;' +
            'border-bottom:1px solid var(--border-subtle);background:var(--bg-header);' +
            'flex-shrink:0;font-size:12px;';
        this._epKindBar = kindBar;
        this._epKindFilter = 'all';
        const kindLbl = document.createElement('span');
        kindLbl.textContent = 'Instance filter:';
        kindLbl.style.cssText =
            'font-size:12px;color:var(--fg-muted);font-weight:600;flex-shrink:0;';
        kindBar.appendChild(kindLbl);
        // Top-level ports are deliberately omitted — they get full
        // visualisation on the Port Delays tab; showing them again here
        // just produced visually-duplicate cards.
        // `countKey` is threaded through as `dataset.kindCountKey` so the
        // count-update path can read it back per button.
        this._epKindBtns = this._makeFilterButtons({
            container: kindBar,
            defs: [
                { key: 'all',      label: 'All',     countKey: '',
                  title: 'show every endpoint regardless of kind' },
                { key: 'flipflop', label: 'Flop',    countKey: 'flipflop',
                  title: 'flip-flop endpoints — edge-triggered sequentials. ' +
                      'D is captured at the active clock edge with ' +
                      'setup/hold around the edge.' },
                { key: 'latch',    label: 'Latch',   countKey: 'latch',
                  title: 'latch endpoints — level-sensitive sequentials. ' +
                      'Transparent while the enable is asserted; D is ' +
                      'captured at the closing edge. Time-borrowing ' +
                      'capable.' },
                { key: 'macro',    label: 'Macro',   countKey: 'macro',
                  title: 'macro / memory endpoints — black-box cells with ' +
                      'liberty-defined setup/hold. Often have many input ' +
                      'pins sharing one clock.' },
                { key: 'stdcell',  label: 'Stdcell', countKey: 'stdcell',
                  title: 'combinational stdcell endpoints — reached via ' +
                      'set_max_delay / set_min_delay -to without a ' +
                      'sequential capture. Timed against an explicit ' +
                      'delay budget.' },
                { key: 'clock_gate', label: 'Clock Gate',
                  countKey: 'clock_gate',
                  title: 'integrated clock-gating cells (Liberty ' +
                      'clock_gating_integrated_cell attribute). The EN ' +
                      'pin has a setup/hold check against CK so the ' +
                      'gated output GCK only pulses when EN was sampled ' +
                      'cleanly. Click an instance to see the gating ' +
                      'waveform.' },
            ],
            initialKey: 'all',
            datasetPrefix: 'kind',
            extras: ['countKey'],
            onSelect: (key) => {
                this._epKindFilter = key;
                this._loadEndpointList({ reset: true });
            },
        });

        // Clock-domain checkbox dropdown. Hidden until the load path
        // sees ≥ 2 clocks. Defaults to every clock checked, so a
        // fresh load behaves identically to the unfiltered case until
        // the user explicitly narrows the set.
        this._epClockFilter = this._makeClockCheckboxFilter({
            container: kindBar,
            title: 'show only endpoints whose capture clock matches one ' +
                'of the selected domains. Default: every clock checked. ' +
                'set_clock_groups -asynchronous pairs are still listed ' +
                'under their own domain.',
            onChange: () => this._loadEndpointList({ reset: true }),
        });

        container.appendChild(kindBar);

        // Two stacked containers — list and detail — that swap visibility.
        // Keeping both alive (rather than re-rendering on every nav) keeps
        // the cards are collapsible-in-place (card-as-detail), so there is
        // no separate detail pane; the list is the only view. Each instance
        // card carries its own ▶/▼ toggle and lazy-fetches its full timing
        // detail when first expanded.
        const listArea = document.createElement('div');
        // `scrollbar-gutter: stable` reserves space for the vertical
        // scrollbar regardless of whether content overflows. Without
        // it, expanding/collapsing a card adds/removes the scrollbar
        // and the resulting clientWidth jump retriggers the
        // ResizeObserver below — flicker loop.
        listArea.style.cssText = 'flex:1;overflow-y:auto;padding:8px;'
            + 'background:var(--bg-main);scrollbar-gutter:stable;';
        this._epListArea = listArea;
        container.appendChild(listArea);
        // Legacy alias retained for code that still reads _epResultArea
        // (e.g. nested-card width calculations in single-pin diagrams).
        this._epResultArea = listArea;

        // Re-draw endpoint instance cards (and their multi-lane timing
        // diagrams when expanded) on width change. Replays every cached
        // batch through `_renderEndpointListBatch` so paginated state
        // is preserved across the redraw, then re-clicks any card the
        // user had expanded before the resize so its detail is re-built
        // at the new width.
        this._installResponsiveRerender('endpointList', listArea, () => {
            const all = this._epListAll;
            if (!Array.isArray(all) || all.length === 0) return;
            const expandedKeys = [];
            for (const card of listArea.querySelectorAll('.sdc-ep-card')) {
                if (card._epExpanded && card.dataset.epKey) {
                    expandedKeys.push(card.dataset.epKey);
                }
            }
            this._renderEndpointListBatch(all, /*append=*/false);
            for (const key of expandedKeys) {
                const card = listArea.querySelector(
                    '.sdc-ep-card[data-ep-key="' + CSS.escape(key) + '"]');
                const header = card
                    && card.querySelector('.sdc-ep-card-header');
                if (header) header.click();
            }
        });

        listArea.innerHTML =
            '<div style="padding:24px;color:var(--fg-muted);font-style:italic;">' +
            'Click "List endpoints" or type a glob filter and press Enter to ' +
            'list every timing endpoint the analyzer is tracking ' +
            '(flip-flops, latches, macro pins, combinational endpoints).</div>';
    }

    // Update the kind-filter button labels with per-kind counts. Called
    // every time we get a fresh kinds_total payload from the backend.
    _updateEpKindCounts(kindsTotal) {
        kindsTotal = kindsTotal || {};
        let totalAll = 0;
        for (const [k, btn] of Object.entries(this._epKindBtns || {})) {
            const ck = btn.dataset.kindCountKey;
            const lbl = btn.dataset.kindLabel;
            if (!ck) continue;
            const n = +(kindsTotal[ck] || 0);
            totalAll += n;
            btn.textContent = `${lbl} (${n})`;
        }
        if (this._epKindBtns && this._epKindBtns.all) {
            this._epKindBtns.all.textContent = `All (${totalAll})`;
        }
    }

    // Fetch (or extend) the endpoint list. opts:
    //   reset: true on a new query (clears list), false for pagination
    //   silent: don't show the "Loading…" placeholder (used by scroll
    //           pagination so the existing list stays visible)
    async _loadEndpointList(opts) {
        opts = opts || {};
        const reset = !!opts.reset;
        const pattern = (this._epInput && this._epInput.value.trim()) || '';
        const kind = this._epKindFilter || 'all';
        const clock = this._epClockFilter
            ? this._epClockFilter.getValue() : 'all';
        if (this._epListLoading) return;
        if (!reset && this._epListAll &&
            this._epListAll.length >= (this._epListTotal || 0)) return;

        const offset = reset ? 0 : (this._epListAll ? this._epListAll.length : 0);
        const token = (this._epListToken || 0) + 1;
        if (reset) {
            this._epListToken = token;
            this._epListAll   = [];
            this._epListTotal = 0;
            this._epListPattern = pattern;
            this._epListKind    = kind;
            this._epListArea.innerHTML =
                '<div style="padding:12px;color:var(--fg-muted);font-style:italic;">' +
                (pattern
                    ? `Searching endpoints for "${pattern}"…`
                    : 'Listing endpoints…') +
                '</div>';
        } else {
            this._showEpListMoreFooter('Loading more…');
        }
        this._epListLoading = true;
        try {
            await this._app.websocketManager.readyPromise;
            // Endpoint enumeration is the slowest SDC call — on a
            // million-instance design the first walk can take well
            // over a minute as STA builds the timing graph + endpoint
            // cache. Allow up to 5 minutes; subsequent paginated
            // batches hit the cache and finish in milliseconds.
            const data = await this._requestWithTimeout({
                type: 'sdc_endpoint_list',
                pattern, kind, clock,
                offset,
                limit: this._batchSize(),
            }, /*timeoutMs=*/300000);
            // Stale token (user issued a newer query while we were waiting).
            if (reset && token !== this._epListToken) return;
            const more = data.endpoints || [];
            this._epListAll = (this._epListAll || []).concat(more);
            if (typeof data.total === 'number') this._epListTotal = data.total;
            // Instance-deduped count from the backend. Drives the
            // footer message; pin-count `total` still drives
            // pagination (offset/limit address pins).
            if (typeof data.total_instances === 'number') {
                this._epListTotalInstances = data.total_instances;
            }
            this._epListTimeUnit = data.time_unit || 'ns';
            // Per-kind totals only refresh on the first batch of a query
            // (subsequent batches share the same totals).
            if (reset) {
                this._updateEpKindCounts(data.kinds_total);
                if (this._epClockFilter) {
                    this._epClockFilter.populate(data.clocks_total || {});
                }
                this._epKindBar.style.display = 'flex';
                this._epListBtn.style.display = 'none';
                if (this._epRefreshBtn) this._epRefreshBtn.style.display = '';
            }
            this._renderEndpointListBatch(more, /* append */ !reset);
            if (!this._epListScrollHandler) this._installEpListInfiniteScroll();
            // If the rendered cards don't fill the viewport, the scroll
            // listener will never fire and the user is stuck — they
            // can't scroll down, so they'd never trigger another
            // pagination request. Top up until the area is scrollable
            // or we've drained the dataset. Ya have to wait one tick
            // so layout settles before measuring scrollHeight.
            this._maybeTopUpEpList();
        } catch (e) {
            console.error('[SDC] endpoint list error:', e);
            if (reset) {
                this._showLoadError(this._epListArea, 'endpoints', e, () => {
                    // Retry with the same query the user originally
                    // submitted. _loadEndpointList({reset:true}) wipes
                    // the in-flight loading flag itself.
                    this._epListLoading = false;
                    this._loadEndpointList({ reset: true });
                });
            } else {
                const msg = e && e.message ? e.message : String(e);
                this._showEpListMoreFooter(`Failed to load more: ${msg}`);
            }
        } finally {
            this._epListLoading = false;
        }
    }

    // ── Endpoint list rendering: grouped instance cards ──────────────────
    //
    // Each card represents ONE instance (or top-level port) and aggregates
    // all of its endpoint pins. Cards are collapsed by default; clicking
    // the ▶ toggle expands a card in place, fetches full timing detail
    // for every endpoint pin on the instance via `sdc_endpoint`, and
    // renders one sub-card per clock domain (since pins on a flop with
    // async preset/clear may sit on different clocks).
    //
    // When grouping by instance, multiple pins (e.g. D + SET_n on a flop,
    // or DAT[31:0] on a macro) share one card — the user sees what's
    // happening at the *instance* level rather than chasing N separate
    // rows for the same physical block.
    //
    // Cross-batch case: if a paginated fetch lands more endpoints for an
    // instance whose card is already on screen, the existing card's
    // pin list extends and (if expanded) re-fetches its body.
    _renderEndpointListBatch(entries, append) {
        if (!append) {
            this._epListArea.innerHTML = '';
            this._epCardsByKey = new Map();  // group-key → card element
        } else {
            const f = this._epListArea.querySelector('.sdc-ep-list-footer');
            if (f) f.remove();
        }
        if (!append && (!entries || entries.length === 0)) {
            const none = document.createElement('div');
            none.style.cssText =
                'padding:12px;color:var(--fg-muted);font-style:italic;';
            const pat = this._epListPattern;
            none.textContent = pat
                ? `No endpoints match "${pat}".`
                : 'No timing endpoints found.';
            this._epListArea.appendChild(none);
            return;
        }
        if (!append) {
            const hdr = document.createElement('div');
            hdr.style.cssText =
                'padding:4px 8px;margin-bottom:6px;font-size:12px;color:var(--fg-muted);' +
                'background:var(--bg-header);border:1px solid var(--border-subtle);' +
                'border-radius:3px;';
            const total = this._epListTotal || 0;
            const pat = this._epListPattern;
            const kind = this._epListKind;
            hdr.textContent =
                `${total} endpoint${total === 1 ? '' : 's'}` +
                (kind && kind !== 'all' ? ` (${kind})` : '') +
                (pat ? ` matching "${pat}"` : '') +
                ' — click ▶ on any row to expand its timing detail in place.';
            this._epListArea.appendChild(hdr);
        }

        // Group entries by (instance) for non-port kinds, by (name) for
        // ports — every port is its own card even when "the instance"
        // would be the top design.
        const groupKey = (e) => e.kind === 'port'
            ? `port:${e.name}`
            : (e.instance ? `inst:${e.instance}` : `pin:${e.name}`);

        for (const e of entries) {
            const key = groupKey(e);
            let card = this._epCardsByKey.get(key);
            if (!card) {
                card = this._makeEndpointInstanceCard({
                    key,
                    kind:     e.kind,
                    instance: e.instance || null,
                    cell:     e.cell || null,
                    pins:     [e],
                    clocks:   new Set(e.clocks || []),
                });
                this._epCardsByKey.set(key, card);
                const footer = this._epListArea.querySelector('.sdc-ep-list-footer');
                if (footer) this._epListArea.insertBefore(card, footer);
                else        this._epListArea.appendChild(card);
            } else {
                // Cross-batch: extend the existing card's pin list.
                card._epGroup.pins.push(e);
                for (const c of (e.clocks || [])) card._epGroup.clocks.add(c);
                this._refreshEndpointInstanceCardHeader(card);
                // If currently expanded with a rendered body, mark it
                // stale so the next collapse-then-expand re-fetches.
                if (card._epExpanded) card._epBodyStale = true;
            }
        }
        // Pagination footer. Counts use *instances* (cards on screen)
        // rather than pins — under a kind filter (e.g. Macro) one
        // instance can contribute many endpoint pins, so the
        // pin-based "Loaded 1800 of 6728" was meaningless next to
        // the 76 macro cards actually rendered. We dedupe the
        // already-loaded entries on instance path; the backend ships
        // the matching unique-instance total in `total_instances`.
        const loadedPins = (this._epListAll && this._epListAll.length) || 0;
        const totalPins  = this._epListTotal || loadedPins;
        const loadedInsts = (() => {
            const seen = new Set();
            for (const e of (this._epListAll || [])) {
                seen.add(e.instance || e.name);
            }
            return seen.size;
        })();
        const totalInsts = (typeof this._epListTotalInstances === 'number')
            ? this._epListTotalInstances : loadedInsts;
        const word = totalInsts === 1 ? 'instance' : 'instances';
        if (totalPins > loadedPins) {
            this._showEpListMoreFooter(
                `Loaded ${loadedInsts} of ${totalInsts} ${word}` +
                ` — scroll for more`);
        } else if (totalInsts > 1) {
            this._showEpListMoreFooter(
                `All ${totalInsts} ${word} loaded`);
        }
    }

    // Build a collapsed instance card. Header summarises the group; the
    // body is empty until the user clicks ▶ for the first time, at which
    // point we lazy-fetch full pin detail and render in place.
    _makeEndpointInstanceCard(group) {
        const card = document.createElement('div');
        card.className = 'sdc-ep-card';
        card.dataset.epKey = group.key;
        card._epGroup = group;
        card._epExpanded = false;
        card._epBodyStale = false;
        card.style.cssText =
            'border:1px solid var(--border-subtle);border-radius:4px;' +
            'margin-bottom:4px;background:var(--bg-input);';

        const header = document.createElement('div');
        header.className = 'sdc-ep-card-header';
        header.style.cssText =
            'display:flex;align-items:center;gap:8px;padding:4px 8px;' +
            'cursor:pointer;font-size:12px;user-select:none;';
        header.addEventListener('mouseenter',
            () => { header.style.background = 'var(--bg-hover)'; });
        header.addEventListener('mouseleave',
            () => { header.style.background = ''; });
        card.appendChild(header);

        const body = document.createElement('div');
        body.className = 'sdc-ep-card-body';
        body.style.cssText =
            'display:none;border-top:1px solid var(--border-subtle);' +
            'padding:6px 8px;background:var(--bg-main);';
        card.appendChild(body);

        const arrow = document.createElement('span');
        arrow.className = 'sdc-ep-arrow';
        arrow.style.cssText =
            'font-size:11px;color:var(--fg-muted);width:10px;flex-shrink:0;';
        arrow.textContent = '▶';
        header.appendChild(arrow);

        // Build the rest of the header from group state — extracted so
        // cross-batch updates can refresh just the dynamic parts (count
        // and clock list) without rebuilding the whole header.
        this._refreshEndpointInstanceCardHeader(card);

        header.addEventListener('click', async () => {
            card._epExpanded = !card._epExpanded;
            arrow.textContent = card._epExpanded ? '▼' : '▶';
            body.style.display = card._epExpanded ? 'block' : 'none';
            if (card._epExpanded && (!card._epBodyLoaded || card._epBodyStale)) {
                card._epBodyLoaded = true;
                card._epBodyStale = false;
                await this._renderEndpointInstanceBody(body, card._epGroup);
            }
        });

        return card;
    }

    _refreshEndpointInstanceCardHeader(card) {
        const group = card._epGroup;
        const header = card.querySelector('.sdc-ep-card-header');
        // Wipe everything after the arrow and rebuild.
        const arrow = header.firstChild;
        header.innerHTML = '';
        header.appendChild(arrow);

        const KIND_BADGE = {
            flipflop: 'background:var(--sdc-master-bg);color:var(--sdc-master-fg);',
            latch:    'background:var(--sdc-mcp-bg);color:var(--sdc-mcp-fg);',
            macro:    'background:var(--sdc-async-bg);color:var(--sdc-async-fg);',
            stdcell:  'background:var(--sdc-pd-bg);color:var(--sdc-pd-fg);',
            clock_gate: 'background:var(--sdc-input-bg);color:var(--sdc-input-fg);',
        };
        const KIND_LABEL = {
            flipflop: 'FLOP', latch: 'LATCH', macro: 'MACRO', stdcell: 'STDCELL',
            clock_gate: 'ICG',
        };
        // Plain-English explainer per kind so a user new to STA knows what
        // the badge means without reading the rest of the card.
        const KIND_TIP = {
            flipflop: 'flip-flop endpoint — edge-triggered sequential. ' +
                'D is captured at the active clock edge with setup/hold ' +
                'around the edge.',
            latch:    'latch endpoint — level-sensitive sequential. ' +
                'Transparent while the enable is asserted; D is captured ' +
                'at the closing edge. Time-borrowing capable.',
            macro:    'macro / memory endpoint — black-box cell with ' +
                'liberty-defined setup/hold. Often has many input pins ' +
                'sharing one clock.',
            stdcell:  'combinational stdcell endpoint — reached via ' +
                'set_max_delay / set_min_delay -to without a sequential ' +
                'capture. Timed against an explicit delay budget.',
            clock_gate: 'integrated clock-gating cell. Liberty ' +
                '`clock_gating_integrated_cell` attribute set. EN must ' +
                'satisfy a setup/hold check against CK so the gated ' +
                'output GCK only pulses when EN was sampled cleanly.',
        };
        const badge = document.createElement('span');
        badge.className = 'sdc-ep-kind';
        badge.style.cssText =
            'font-size:11px;padding:1px 5px;border-radius:3px;font-weight:600;' +
            'min-width:42px;text-align:center;flex-shrink:0;' +
            (KIND_BADGE[group.kind] || 'background:var(--bg-input);color:var(--fg-muted);');
        badge.textContent = KIND_LABEL[group.kind] || (group.kind || '?').toUpperCase();
        if (KIND_TIP[group.kind]) badge.title = KIND_TIP[group.kind];
        header.appendChild(badge);

        const nameSpan = document.createElement('span');
        nameSpan.style.cssText =
            'font-family:monospace;font-weight:600;color:var(--fg-primary);' +
            'flex:1;min-width:0;' + TRUNCATE_PATH_CSS;
        const display = group.kind === 'port'
            ? (group.pins[0] && group.pins[0].name) || ''
            : group.instance || (group.pins[0] && group.pins[0].name) || '';
        nameSpan.textContent = display;
        nameSpan.title = display;
        header.appendChild(nameSpan);

        if (group.cell) {
            const cellSpan = document.createElement('span');
            cellSpan.style.cssText =
                'font-size:11px;color:var(--fg-muted);font-family:monospace;' +
                'white-space:nowrap;flex-shrink:0;';
            cellSpan.textContent = group.cell;
            header.appendChild(cellSpan);
        }

        // Endpoint count — distinguishes "macro with 32 endpoint pins" from
        // "flop with 1 endpoint pin" at a glance.
        const cnt = group.pins.length;
        if (cnt > 1) {
            const cntSpan = document.createElement('span');
            cntSpan.style.cssText =
                'font-size:11px;color:var(--fg-muted);white-space:nowrap;flex-shrink:0;';
            cntSpan.textContent = `${cnt} endpoints`;
            header.appendChild(cntSpan);
        }

        if (group.clocks.size > 0) {
            const clkSpan = document.createElement('span');
            clkSpan.style.cssText =
                'font-size:11px;color:var(--fg-muted);white-space:nowrap;flex-shrink:0;';
            const clocks = [...group.clocks];
            clkSpan.textContent = '⏱ ' + clocks.join(', ');
            clkSpan.title = clocks.join(', ');
            header.appendChild(clkSpan);
        }
    }

    // Lazy-fetch all endpoint pins on this instance and render the body
    // in place. Triggered the first time the user clicks ▶ on a card,
    // and re-triggered when cross-batch pagination adds more pins to
    // the group while it's already expanded.
    async _renderEndpointInstanceBody(body, group) {
        // Clock-gate cells get a dedicated waveform diagram (CK + EN +
        // optional LATCH + GCK). Render directly from the group data —
        // no network fetch needed, the endpoint-list response already
        // carries `clock_gate_flavor` + the CK / EN / OUT pin paths.
        if (group.kind === 'clock_gate') {
            this._renderClockGateBody(body, group);
            return;
        }
        body.innerHTML =
            '<div style="padding:8px;color:var(--fg-muted);font-style:italic;">Loading…</div>';
        try {
            // For a port group there's just one pin to fetch by name;
            // for an instance group we ask for everything under the
            // instance path (limit=-1 → no pagination cap), then filter
            // the response to only the pins our endpoint set knows about.
            const targetGlob = group.kind === 'port'
                ? (group.pins[0] && group.pins[0].name) || ''
                : `${group.instance}/*`;
            const data = await this._requestWithTimeout({
                type: 'sdc_endpoint',
                pin:    targetGlob,
                offset: 0,
                limit:  -1,
            });
            const endpointNames = new Set(group.pins.map(p => p.name));
            const allPins = data.pins || [];
            const timeUnit = data.time_unit || 'ns';
            body.innerHTML = '';
            if (allPins.length === 0) {
                const none = document.createElement('div');
                none.style.cssText =
                    'padding:8px;color:var(--fg-muted);font-style:italic;font-size:12px;';
                none.textContent = 'No timing data available for this instance.';
                body.appendChild(none);
                return;
            }
            // Tag each pin with whether it's an endpoint of the timing graph
            // (drives the "endpoint" highlight) and partition out clock pins
            // (CK/CLK) — those get summarised in the card header rather
            // than drawn as a lane. Supply pins (VDD/VSS/well) carry no
            // timing semantics and are skipped entirely.
            const SUPPLY_DIRS = new Set(['power', 'ground', 'well']);
            const dataPins = [];
            const clockPins = [];
            for (const p of allPins) {
                if (SUPPLY_DIRS.has(p.direction)) continue;
                p._isEndpoint = endpointNames.has(p.name);
                if (p.is_clock_pin) clockPins.push(p);
                else                dataPins.push(p);
            }
            if (clockPins.length > 0) {
                body.appendChild(this._renderEndpointClockPinSection(
                    clockPins, timeUnit, group.kind, group.instance));
            }

            // Capture-side clocks: only those that domain at least one
            // endpoint pin (the pin Search::endpoints() flagged). On a
            // flop instance with cross-clock paths the Q pin's
            // clockDomains() can include downstream launch clocks; we
            // don't want those to spawn their own per-clock sub-cards
            // because the user is on this card to debug the *capture*
            // side. When no endpoint pin reports a clock (combinational
            // stdcell endpoints reached via set_max_delay -to) we fall
            // back to "no filter" so the user still sees something.
            const captureClockNames = new Set();
            for (const p of allPins) {
                if (!endpointNames.has(p.name)) continue;
                for (const c of (p.clocks || [])) {
                    if (c && c.name) captureClockNames.add(c.name);
                }
            }
            const isCaptureClock = (name) =>
                captureClockNames.size === 0 || captureClockNames.has(name);

            // Group data pins by clock domain. Pins without any clock (e.g.
            // const-driven async pins, all-tied-off latches) bucket
            // separately so we can render them as a compact inline list
            // rather than an empty "(no clock)" sub-card with no diagram.
            const byClock = new Map();
            const noClockPins = [];
            for (const p of dataPins) {
                const clocks = (p.clocks || []).filter(
                    c => c && isCaptureClock(c.name));
                if (clocks.length === 0) {
                    noClockPins.push(p);
                    continue;
                }
                for (const c of clocks) {
                    const key = c.name || '(no clock)';
                    if (!byClock.has(key)) byClock.set(key, { clock: c, pins: [] });
                    byClock.get(key).pins.push(p);
                }
            }
            for (const [, sub] of byClock) {
                body.appendChild(this._renderEndpointClockSubcard(
                    sub.clock, sub.pins, timeUnit, group.kind,
                    group.instance));
            }
            // No-clock pins: render as a compact pin list with a header
            // explaining why no diagram is drawn. Each name is still
            // clickable through to the inspector.
            if (noClockPins.length > 0) {
                body.appendChild(this._renderEndpointNoClockSection(
                    noClockPins, group.instance));
            }

            // Deduped exception summary for the whole card. Walks every
            // pin (including non-endpoint instance pins) so an exception
            // touching e.g. the Q output is still surfaced.
            const excSection = this._renderEndpointInstanceExceptions(
                allPins, timeUnit);
            if (excSection) body.appendChild(excSection);
        } catch (e) {
            const err = document.createElement('div');
            err.style.cssText =
                'padding:8px;color:var(--fg-muted);font-style:italic;font-size:12px;';
            err.textContent = `Error: ${(e && e.message) || e}`;
            body.innerHTML = '';
            body.appendChild(err);
        }
    }

    // Render one per-clock sub-card inside an expanded instance card.
    // The clock-name header is sticky so when a macro has many endpoint
    // pins and the user scrolls down through them, the clock context
    // stays pinned at the top of the sub-card.
    //
    // The body stacks one existing per-pin clock-diagram-card per
    // endpoint pin in this clock domain. (A future iteration will
    // collapse this into a true multi-lane shared-CLK diagram with one
    // lane per pin — see the plan doc.)
    _renderEndpointClockSubcard(clkInfo, pins, timeUnit, instanceKind,
                                instancePath) {
        const sub = document.createElement('div');
        sub.className = 'sdc-ep-clock-subcard';
        sub.style.cssText =
            'margin-bottom:8px;border:1px solid var(--border-subtle);' +
            'border-radius:4px;background:var(--bg-input);overflow:hidden;';
        const hdr = document.createElement('div');
        hdr.className = 'sdc-ep-clock-header';
        hdr.style.cssText =
            'padding:4px 8px;background:var(--bg-header);' +
            'border-bottom:1px solid var(--border-subtle);' +
            'font-size:12px;color:var(--fg-primary);font-weight:600;' +
            'display:flex;align-items:center;gap:8px;' +
            'position:sticky;top:0;z-index:1;';
        const clkLabel = document.createElement('span');
        clkLabel.style.cssText = 'font-family:monospace;';
        clkLabel.textContent = `on ${clkInfo.name || '(no clock)'}`;
        hdr.appendChild(clkLabel);
        if (clkInfo.period != null) {
            const summary = document.createElement('span');
            summary.style.cssText =
                'font-size:11px;color:var(--fg-muted);font-weight:400;';
            const parts = [
                `T=${clkInfo.period.toPrecision(3)}${timeUnit}`,
                this._periodToFreq(clkInfo.period),
            ];
            if (clkInfo.uncertainty_setup != null) {
                parts.push(`${TIMING_LABELS.setup_unc.short}=${clkInfo.uncertainty_setup.toPrecision(3)}${timeUnit}`);
            }
            if (clkInfo.uncertainty_hold != null) {
                parts.push(`${TIMING_LABELS.hold_unc.short}=${clkInfo.uncertainty_hold.toPrecision(3)}${timeUnit}`);
            }
            summary.textContent = parts.join('  ');
            hdr.appendChild(summary);
        }
        const cnt = document.createElement('span');
        cnt.style.cssText =
            'margin-left:auto;font-size:11px;color:var(--fg-muted);font-weight:400;';
        cnt.textContent = `${pins.length} pin${pins.length === 1 ? '' : 's'}`;
        hdr.appendChild(cnt);
        sub.appendChild(hdr);

        const body = document.createElement('div');
        body.style.cssText = 'padding:6px 8px;display:flex;flex-direction:column;gap:6px;';

        // Bus-expansion state lives on the sub-card so each clock domain
        // tracks its own expanded buses. Click on a bus-collapsed lane
        // adds the bus root to this set and triggers a fresh diagram
        // render in place.
        const expandedBuses = new Set();
        const renderDiagram = () => {
            // Wipe any previous diagram (re-render path) but keep the
            // sub-card header in place.
            body.innerHTML = '';
            const diagram = this._renderEndpointMultiLaneDiagram(
                clkInfo, pins, timeUnit, {
                    instanceKind,
                    instancePath,
                    expandedBuses,
                    onBusExpand: (busRoot) => {
                        expandedBuses.add(busRoot);
                        renderDiagram();
                    },
                });
            if (diagram) {
                body.appendChild(diagram);
            } else {
                // Fallback: per-pin diagrams when multi-lane couldn't draw
                // (e.g. clock with no waveform data).
                for (const p of pins) {
                    const pinRow = document.createElement('div');
                    pinRow.style.cssText =
                        'border:1px solid var(--border-subtle);border-radius:3px;' +
                        'background:var(--bg-main);overflow:hidden;';
                    const lbl = document.createElement('div');
                    lbl.style.cssText =
                        'padding:3px 8px;font-family:monospace;font-size:12px;' +
                        'color:var(--fg-primary);background:var(--bg-input);' +
                        'border-bottom:1px solid var(--border-subtle);';
                    lbl.textContent = this._pinLeafName(p.name, instancePath);
                    lbl.title = p.name;
                    this._linkifyPin(lbl, p, 'name');
                    pinRow.appendChild(lbl);
                    const clkForPin = (p.clocks || []).find(
                        c => c.name === clkInfo.name) || clkInfo;
                    if (clkForPin && clkForPin.period != null) {
                        pinRow.appendChild(this._renderClockDiagramCard(clkForPin, timeUnit));
                    }
                    body.appendChild(pinRow);
                }
            }
        };
        renderDiagram();

        sub.appendChild(body);
        return sub;
    }

    // ── Clock-gate (ICG) endpoint body ─────────────────────────────────
    //
    // For Liberty `clock_gating_integrated_cell` endpoints, the EN pin
    // has a setup/hold check against CK and the cell's gated output
    // (GCK) only pulses when EN was sampled cleanly. The body renders:
    //   1) A header line naming the cell + flavor + pin roles.
    //   2) A waveform SVG with CK / EN / (optional LATCH) / GCK rows
    //      illustrating the gating semantics. The traces are
    //      schematic — drawn to teach the gating contract, not to
    //      reflect runtime values (which would require simulation).
    //   3) Three flavor-specific behaviour notes the user can reference
    //      when interpreting setup/hold violations on EN.
    _renderClockGateBody(body, group) {
        body.innerHTML = '';
        const e = (group.pins && group.pins[0]) || {};
        const flavor = e.clock_gate_flavor || 'other';
        const ckPin = e.clock_gate_ck_pin || '';
        const enPin = e.clock_gate_en_pin || '';
        const outPin = e.clock_gate_out_pin || '';
        const cellName = group.cell || '?';
        // Numeric setup/hold values from the cell's Liberty timing
        // arcs (gatedClockSetup / gatedClockHold), evaluated at the
        // library's default operating conditions. Null when the cell
        // has no check arcs (e.g. mux/other flavor) or when the
        // CheckTimingModel evaluation failed.
        const setupVal = (typeof e.clock_gate_setup === 'number')
            ? e.clock_gate_setup : null;
        const holdVal = (typeof e.clock_gate_hold === 'number')
            ? e.clock_gate_hold : null;

        // Header strip.
        const header = document.createElement('div');
        header.style.cssText =
            'padding:6px 8px;display:flex;flex-wrap:wrap;gap:6px;'
            + 'align-items:baseline;font-size:12px;line-height:1.5;';
        const FLAVOR_BADGE = {
            latch_posedge: 'background:var(--sdc-master-bg);'
                + 'color:var(--sdc-master-fg);',
            latch_negedge: 'background:var(--sdc-mcp-bg);'
                + 'color:var(--sdc-mcp-fg);',
            other: 'background:var(--sdc-pd-bg);color:var(--sdc-pd-fg);',
        };
        const FLAVOR_LABEL = {
            latch_posedge: 'latch_posedge',
            latch_negedge: 'latch_negedge',
            other: 'mux / other',
        };
        const FLAVOR_TIP = {
            latch_posedge:
                'Active edge is CK rising. Internal latch is transparent ' +
                'while CK is LOW (samples EN), opaque while CK is HIGH ' +
                '(holds the previously-sampled value). EN must satisfy the '
                + 'setup/hold window around the CK rising edge.',
            latch_negedge:
                'Active edge is CK falling. Internal latch is transparent ' +
                'while CK is HIGH, opaque while CK is LOW. EN must satisfy '
                + 'the setup/hold window around the CK falling edge.',
            other:
                'Mux- or AND-style integrated clock gate without an '
                + 'internal latch. EN gates CK directly — vulnerable to '
                + 'glitches if EN transitions while CK is in its active '
                + 'level. Real designs almost always use a latched flavor.',
        };
        const flavorBadge = document.createElement('span');
        flavorBadge.style.cssText =
            'font-size:11px;padding:1px 6px;border-radius:3px;font-weight:600;'
            + (FLAVOR_BADGE[flavor] || FLAVOR_BADGE.other);
        flavorBadge.textContent = FLAVOR_LABEL[flavor] || flavor;
        flavorBadge.title = FLAVOR_TIP[flavor] || '';
        header.appendChild(flavorBadge);

        const cellSpan = document.createElement('span');
        cellSpan.style.cssText =
            'font-family:monospace;font-weight:600;color:var(--fg-primary);';
        cellSpan.textContent = cellName;
        cellSpan.title = `Liberty cell ${cellName} carries the `
            + 'clock_gating_integrated_cell attribute.';
        header.appendChild(cellSpan);

        const addPinChip = (role, pinPath) => {
            if (!pinPath) return;
            const chip = document.createElement('span');
            chip.style.cssText =
                'font-family:monospace;font-size:11px;'
                + 'color:var(--fg-secondary);';
            chip.innerHTML = `<span style="color:var(--fg-muted)">${role}=</span>`;
            const path = document.createElement('span');
            path.textContent = pinPath;
            path.title = pinPath;
            chip.appendChild(path);
            header.appendChild(chip);
        };
        addPinChip('CK', ckPin);
        addPinChip('EN', enPin);
        addPinChip('GCK', outPin);
        body.appendChild(header);

        // Waveform diagram — schematic SVG illustrating the gating
        // contract. Reads the parent panel's clientWidth so the SVG
        // resizes via the existing _installResponsiveRerender hook.
        const period = (e.clocks && e.clocks[0] && e.clocks[0].period)
            || 10.0;
        const timeUnit = this._timeUnit || 'ns';
        const svgWidth = Math.max(
            520, (this._epListArea && this._epListArea.clientWidth - 60) || 600);
        body.appendChild(this._renderClockGateDiagram({
            flavor, cellName, ckPin, enPin, outPin,
            period, timeUnit, width: svgWidth,
            setupVal, holdVal,
        }));

        // Legend strip — names the colours used in the diagram so a
        // reader doesn't have to hover the bands to figure out which
        // is setup and which is hold. Each swatch carries a tooltip
        // summarising the corresponding Liberty timing arc.
        const legend = document.createElement('div');
        legend.style.cssText =
            'padding:4px 8px;display:flex;flex-wrap:wrap;'
            + 'gap:12px 16px;font-size:11px;color:var(--fg-secondary);'
            + 'align-items:center;';
        const swatch = (color, alpha, label, tip) => {
            const item = document.createElement('span');
            item.style.cssText = 'display:inline-flex;align-items:center;'
                + 'gap:6px;cursor:default;';
            item.title = tip;
            const sw = document.createElement('span');
            sw.style.cssText = 'display:inline-block;width:14px;height:10px;'
                + `background:${color};opacity:${alpha};`
                + 'border:1px solid var(--border-subtle);border-radius:2px;';
            item.appendChild(sw);
            const lbl = document.createElement('span');
            lbl.textContent = label;
            item.appendChild(lbl);
            return item;
        };
        // Format helper — render a numeric value with the SDC tab's
        // active time-unit, or "library default — n/a" when the cell
        // has no check arc for this role.
        const fmtTime = (v) =>
            (v == null) ? null
                        : `${(+v).toPrecision(3)}${this._timeUnit || 'ns'}`;
        const setupFmt = fmtTime(setupVal);
        const holdFmt  = fmtTime(holdVal);

        legend.appendChild(swatch(
            'var(--sdc-wf-setup-unc)', 0.6,
            setupFmt ? `setup ${setupFmt}` : 'setup band',
            'Orange — EN must be stable BEFORE the active CK edge. '
            + (setupFmt
                ? `Cell-Liberty default: ${setupFmt} `
                  + '(read from the `clock_gating_setup` timing arc at '
                  + 'the library\'s default operating conditions). A '
                  + '`set_clock_gating_check -setup <value>` SDC '
                  + 'override would replace this — overrides aren\'t '
                  + 'enumerable yet (private OpenSTA map).'
                : 'No `clock_gating_setup` arc on this cell — common '
                  + 'for mux/other flavor ICGs without an internal '
                  + 'latch.')));
        legend.appendChild(swatch(
            'var(--sdc-wf-hold-unc)', 0.6,
            holdFmt ? `hold ${holdFmt}` : 'hold band',
            'Cyan — EN must remain stable AFTER the active CK edge. '
            + (holdFmt
                ? `Cell-Liberty default: ${holdFmt} `
                  + '(read from the `clock_gating_hold` timing arc at '
                  + 'the library\'s default operating conditions). A '
                  + '`set_clock_gating_check -hold <value>` SDC '
                  + 'override would replace this — overrides aren\'t '
                  + 'enumerable yet (private OpenSTA map).'
                : 'No `clock_gating_hold` arc on this cell — common '
                  + 'for mux/other flavor ICGs without an internal '
                  + 'latch.')));
        // Active-edge guide swatch — dashed magenta line.
        const guideItem = document.createElement('span');
        guideItem.style.cssText = 'display:inline-flex;align-items:center;'
            + 'gap:6px;cursor:default;';
        guideItem.title = (flavor === 'latch_negedge'
            ? 'Active CK falling edge — EN sampled here.'
            : 'Active CK rising edge — EN sampled here.');
        const gsw = document.createElement('span');
        gsw.style.cssText = 'display:inline-block;width:14px;height:0;'
            + 'border-top:1.5px dashed var(--sdc-text-setup);';
        guideItem.appendChild(gsw);
        const glabel = document.createElement('span');
        glabel.textContent = 'active CK edge';
        guideItem.appendChild(glabel);
        legend.appendChild(guideItem);
        body.appendChild(legend);

        // Behavior note — reinforce what the diagram shows.
        const note = document.createElement('div');
        note.style.cssText =
            'padding:4px 8px;font-size:11px;color:var(--fg-muted);'
            + 'font-style:italic;line-height:1.4;';
        const haveBoth = setupFmt && holdFmt;
        const haveAny = setupFmt || holdFmt;
        note.textContent = haveBoth
            ? 'EN/LATCH/GCK traces illustrate the gating contract '
              + '(transition timing is schematic, not runtime). The '
              + 'setup and hold bands ARE drawn to scale — their '
              + 'widths are the cell\'s `clock_gating_setup` / '
              + '`clock_gating_hold` Liberty arc values at the '
              + 'library\'s default operating conditions, expressed '
              + 'as a fraction of the clock period. Per-pin SDC '
              + 'overrides via `set_clock_gating_check` are not yet '
              + 'enumerable — the OpenSTA accessor for the override '
              + 'map is private.'
            : haveAny
            ? 'EN/LATCH/GCK traces illustrate the gating contract '
              + '(transition timing is schematic, not runtime). The '
              + (setupFmt ? 'setup' : 'hold') + ' band IS drawn to '
              + 'scale (from the cell\'s Liberty arc at default op '
              + 'conditions); the ' + (setupFmt ? 'hold' : 'setup')
              + ' band is a placeholder width — the cell exposes no '
              + 'readable arc for it (common for mux/other flavor '
              + 'ICGs without an internal latch on that side).'
            : 'EN/LATCH/GCK traces illustrate the gating contract '
              + '(transition timing is schematic, not runtime). '
              + 'This cell has no readable `clock_gating_setup` / '
              + '`clock_gating_hold` arcs — common for mux/other '
              + 'flavor ICGs without an internal latch. Setup/hold '
              + 'band widths shown are placeholder.';
        body.appendChild(note);
    }

    // SVG renderer for the ICG gating waveform. Three flavors:
    //   - latch_posedge: latch transparent on CK low, active edge = CK rise
    //   - latch_negedge: latch transparent on CK high, active edge = CK fall
    //   - other (mux): no latch, EN gates CK directly (glitch-prone)
    //
    // 4 rows: CK, EN, LATCH (omitted for `other`), GCK. Time axis on
    // top covers 3 cycles. Setup/hold bands flank each active edge.
    _renderClockGateDiagram(opts) {
        const SVG_NS = 'http://www.w3.org/2000/svg';
        const flavor = opts.flavor || 'other';
        const cycles = 3;
        const period = opts.period > 0 ? opts.period : 10.0;
        const span = period * cycles;
        const showLatch = flavor !== 'other';
        const rows = showLatch
            ? ['CK', 'EN', 'LATCH', 'GCK']
            : ['CK', 'EN', 'GCK'];
        const LABEL_W = 56;
        const ROW_H = 38;
        const PAD_TOP = 22;       // axis labels
        const PAD_BOTTOM = 10;
        const totalW = Math.max(opts.width || 600, LABEL_W + 200);
        const waveW = totalW - LABEL_W - 8;
        const totalH = PAD_TOP + rows.length * ROW_H + PAD_BOTTOM;
        const svg = this._makeSvg(totalW, totalH);

        // Time → x within wave area.
        const tx = (t) => LABEL_W + (t / span) * waveW;

        // Row Y for the *waveform centerline* of row index.
        const rowY = (idx) => PAD_TOP + idx * ROW_H + ROW_H / 2;
        const rowTop = (idx) => PAD_TOP + idx * ROW_H + 8;
        const rowBot = (idx) => PAD_TOP + (idx + 1) * ROW_H - 4;

        // ── Time axis ────────────────────────────────────────────────
        const axisY = PAD_TOP - 8;
        this._svgLine(svg, LABEL_W, axisY, LABEL_W + waveW, axisY,
                      'var(--canvas-axis)', 1);
        for (let i = 0; i <= cycles; ++i) {
            const x = tx(i * period);
            this._svgLine(svg, x, axisY - 3, x, axisY + 2,
                          'var(--canvas-axis)', 1);
            const label = `${(i * period).toPrecision(3)}${opts.timeUnit}`;
            this._svgText(svg, x, axisY - 6, label,
                          'var(--canvas-label)', 9, 'middle');
        }

        // ── Row labels ──────────────────────────────────────────────
        for (let i = 0; i < rows.length; ++i) {
            this._svgText(svg, 4, rowY(i) + 4, rows[i],
                          'var(--fg-secondary)', 11, 'start');
        }

        // Helper: square-wave path for a given row, from list of (t, level)
        // segments. Each segment: { t0, t1, level } with level ∈ {0,1}.
        const drawSquare = (idx, segments, color) => {
            const top = rowTop(idx);
            const bot = rowBot(idx);
            let pathD = '';
            for (let s = 0; s < segments.length; ++s) {
                const seg = segments[s];
                const x0 = tx(seg.t0);
                const x1 = tx(seg.t1);
                const y = seg.level ? top : bot;
                if (s === 0) {
                    pathD += `M ${x0} ${y} `;
                } else {
                    const prev = segments[s - 1];
                    const prevY = prev.level ? top : bot;
                    if (prev.level !== seg.level) {
                        pathD += `L ${x0} ${prevY} L ${x0} ${y} `;
                    }
                }
                pathD += `L ${x1} ${y} `;
            }
            const path = document.createElementNS(SVG_NS, 'path');
            path.setAttribute('d', pathD);
            path.setAttribute('fill', 'none');
            path.setAttribute('stroke', color);
            path.setAttribute('stroke-width', '1.5');
            svg.appendChild(path);
        };

        // ── CK waveform: 50% duty cycle, three full cycles ───────────
        const ckSegs = [];
        for (let i = 0; i < cycles; ++i) {
            ckSegs.push({ t0: i * period, t1: i * period + period / 2,
                          level: 0 });
            ckSegs.push({ t0: i * period + period / 2, t1: (i + 1) * period,
                          level: 1 });
        }
        drawSquare(0, ckSegs, 'var(--sdc-wf-valid)');

        // Hover area for CK row's title.
        const ckHover = document.createElementNS(SVG_NS, 'rect');
        ckHover.setAttribute('x', LABEL_W);
        ckHover.setAttribute('y', PAD_TOP);
        ckHover.setAttribute('width', waveW);
        ckHover.setAttribute('height', ROW_H);
        ckHover.setAttribute('fill', 'transparent');
        const ckTitle = document.createElementNS(SVG_NS, 'title');
        ckTitle.textContent = `CK = master clock input. Period ${period}`
            + `${opts.timeUnit}. ` + (flavor === 'latch_negedge'
                ? 'Active edge is the falling edge.'
                : 'Active edge is the rising edge.');
        ckHover.appendChild(ckTitle);
        svg.appendChild(ckHover);

        // ── Active-edge markers + setup/hold bands ───────────────────
        const activeIsRising = flavor !== 'latch_negedge';
        // When the backend was able to read the cell's clock-gating
        // setup/hold arcs, draw the bands TO SCALE (value / period)
        // so the diagram is honest about how tight/loose the window
        // actually is. Without numeric values, fall back to fixed
        // schematic widths sized for visibility.
        const SETUP_FRAC_SCHEMATIC = 0.10;
        const HOLD_FRAC_SCHEMATIC = 0.06;
        const haveSetupVal = (typeof opts.setupVal === 'number')
            && period > 0;
        const haveHoldVal  = (typeof opts.holdVal === 'number')
            && period > 0;
        const setupFrac = haveSetupVal
            ? Math.max(0, opts.setupVal / period)
            : SETUP_FRAC_SCHEMATIC;
        const holdFrac = haveHoldVal
            ? Math.max(0, opts.holdVal / period)
            : HOLD_FRAC_SCHEMATIC;
        const setupColor = 'var(--sdc-wf-setup-unc)';
        const holdColor  = 'var(--sdc-wf-hold-unc)';
        for (let i = 0; i < cycles; ++i) {
            const t_edge = activeIsRising
                ? i * period + period / 2  // CK low→high
                : i * period;              // CK high→low (i==0 implies edge at 0)
            // Skip the first edge if it sits at t=0 (off-screen left).
            if (t_edge <= 0) continue;
            const x = tx(t_edge);
            // Vertical guide on every row.
            const guideD = `M ${x} ${PAD_TOP} L ${x} ${rowBot(rows.length - 1) + 4}`;
            const guide = document.createElementNS(SVG_NS, 'path');
            guide.setAttribute('d', guideD);
            guide.setAttribute('stroke', 'var(--sdc-text-setup)');
            guide.setAttribute('stroke-dasharray', '2 2');
            guide.setAttribute('stroke-width', '1');
            guide.setAttribute('fill', 'none');
            svg.appendChild(guide);
            const guideTitle = document.createElementNS(SVG_NS, 'title');
            guideTitle.textContent = activeIsRising
                ? 'CK rising edge — latched(EN) sampled here.'
                : 'CK falling edge — latched(EN) sampled here.';
            guide.appendChild(guideTitle);

            // Setup band (orange) — pre-edge. Tooltip leads with
            // "Setup" so hover-on-band identifies which colour it is
            // without needing the legend below the diagram. When the
            // backend was able to read the cell's `clock_gating_setup`
            // arc, the band is drawn TO SCALE (value/period) and the
            // tooltip reports the actual time; otherwise we fall back
            // to a fixed schematic width sized for visibility.
            const setupRectW = Math.max(
                2, tx(t_edge) - tx(t_edge - period * setupFrac));
            const setupRect = document.createElementNS(SVG_NS, 'rect');
            setupRect.setAttribute('x', x - setupRectW);
            setupRect.setAttribute('y', rowTop(1) - 4);
            setupRect.setAttribute('width', setupRectW);
            setupRect.setAttribute('height', ROW_H - 8);
            setupRect.setAttribute('fill', setupColor);
            setupRect.setAttribute('fill-opacity', '0.35');
            const setupTitle = document.createElementNS(SVG_NS, 'title');
            const sFmt = haveSetupVal
                ? `${(+opts.setupVal).toPrecision(3)}${opts.timeUnit}`
                : null;
            const periodFmt
                = `${(+period).toPrecision(3)}${opts.timeUnit}`;
            setupTitle.textContent = sFmt
                ? 'Setup band (orange) — EN must be stable BEFORE the '
                  + `active CK edge. Cell-Liberty default: ${sFmt} `
                  + `(${(setupFrac * 100).toFixed(1)}% of the `
                  + `${periodFmt} period). Band drawn to scale; from `
                  + 'the `clock_gating_setup` arc at default op '
                  + 'conditions. Override per-pin via '
                  + '`set_clock_gating_check -setup <value>`.'
                : 'Setup band (orange) — EN must be stable BEFORE the '
                  + 'active CK edge. The drawn band width is schematic '
                  + '(' + Math.round(setupFrac * 100)
                  + `% of the ${periodFmt} period for visibility). The `
                  + 'cell has no `clock_gating_setup` arc readable by '
                  + 'this view — common for mux/other flavor ICGs. Set '
                  + 'with `set_clock_gating_check -setup <value>`.';
            setupRect.appendChild(setupTitle);
            svg.appendChild(setupRect);

            // Hold band (cyan) — post-edge.
            const holdRectW = Math.max(
                2, tx(t_edge + period * holdFrac) - tx(t_edge));
            const holdRect = document.createElementNS(SVG_NS, 'rect');
            holdRect.setAttribute('x', x);
            holdRect.setAttribute('y', rowTop(1) - 4);
            holdRect.setAttribute('width', holdRectW);
            holdRect.setAttribute('height', ROW_H - 8);
            holdRect.setAttribute('fill', holdColor);
            holdRect.setAttribute('fill-opacity', '0.35');
            const holdTitle = document.createElementNS(SVG_NS, 'title');
            const hFmt = haveHoldVal
                ? `${(+opts.holdVal).toPrecision(3)}${opts.timeUnit}`
                : null;
            holdTitle.textContent = hFmt
                ? 'Hold band (cyan) — EN must remain stable AFTER the '
                  + `active CK edge. Cell-Liberty default: ${hFmt} `
                  + `(${(holdFrac * 100).toFixed(1)}% of the `
                  + `${periodFmt} period). Band drawn to scale; from `
                  + 'the `clock_gating_hold` arc at default op '
                  + 'conditions. Override per-pin via '
                  + '`set_clock_gating_check -hold <value>`.'
                : 'Hold band (cyan) — EN must remain stable AFTER the '
                  + 'active CK edge. The drawn band width is schematic '
                  + '(' + Math.round(holdFrac * 100)
                  + `% of the ${periodFmt} period for visibility). The `
                  + 'cell has no `clock_gating_hold` arc readable by '
                  + 'this view — common for mux/other flavor ICGs. Set '
                  + 'with `set_clock_gating_check -hold <value>`.';
            holdRect.appendChild(holdTitle);
            svg.appendChild(holdRect);
        }

        // ── EN row: schematic transition. Goes high mid-cycle 1, low
        // mid-cycle 2 so the user sees a gated and an enabled stretch.
        const enHighStart = period * 1.25;
        const enHighEnd   = period * 2.25;
        const enSegs = [
            { t0: 0, t1: enHighStart, level: 0 },
            { t0: enHighStart, t1: enHighEnd, level: 1 },
            { t0: enHighEnd, t1: span, level: 0 },
        ];
        drawSquare(1, enSegs, 'var(--sdc-wf-valid)');
        const enHover = document.createElementNS(SVG_NS, 'rect');
        enHover.setAttribute('x', LABEL_W);
        enHover.setAttribute('y', rowTop(1));
        enHover.setAttribute('width', waveW);
        enHover.setAttribute('height', ROW_H - 8);
        enHover.setAttribute('fill', 'transparent');
        const enTitle = document.createElementNS(SVG_NS, 'title');
        enTitle.textContent = 'EN = enable input. Sampled by the '
            + 'cell\'s internal latch ' + (flavor === 'latch_negedge'
                ? 'on CK high'
                : 'on CK low')
            + ' to avoid glitches when CK is at its active level.';
        enHover.appendChild(enTitle);
        svg.appendChild(enHover);

        // ── LATCH row (latched value of EN). Tracks EN during the
        // transparent half of CK; holds during the opaque half.
        let latchSegs = [];
        if (showLatch) {
            // Sample EN at every transparent → opaque transition. For
            // latch_posedge, transparent when CK low, opaque when CK
            // high. For latch_negedge, the inverse.
            const enValueAt = (t) => {
                for (const s of enSegs) {
                    if (t >= s.t0 && t < s.t1) return s.level;
                }
                return enSegs[enSegs.length - 1].level;
            };
            const sampleT = (i) => activeIsRising
                ? i * period + period / 2  // CK rises here
                : i * period + period;     // CK falls at end of cycle
            // Build latch segments.
            // Initial latched value = EN at t=0 (low).
            let latched = enValueAt(0);
            // Transparent windows pass EN through directly; opaque
            // windows hold the latched value.
            // For latch_posedge: transparent = [i*T, i*T + T/2);
            //                     opaque     = [i*T + T/2, (i+1)*T)
            // For latch_negedge: opaque = [i*T, i*T + T/2);
            //                     transparent = [i*T + T/2, (i+1)*T)
            for (let i = 0; i < cycles; ++i) {
                const tEdge = sampleT(i);
                if (activeIsRising) {
                    // Transparent first half: latch follows EN. Approx as
                    // EN level at the END of the transparent window.
                    latched = enValueAt(tEdge - 0.001);
                    latchSegs.push({ t0: i * period, t1: tEdge,
                                      level: latched });
                    // Opaque half: hold.
                    latchSegs.push({ t0: tEdge, t1: (i + 1) * period,
                                      level: latched });
                } else {
                    // Opaque first half.
                    latchSegs.push({ t0: i * period,
                                      t1: i * period + period / 2,
                                      level: latched });
                    // Transparent second half: latch follows EN.
                    latched = enValueAt((i + 1) * period - 0.001);
                    latchSegs.push({ t0: i * period + period / 2,
                                      t1: (i + 1) * period,
                                      level: latched });
                }
            }
            drawSquare(2, latchSegs, 'var(--sdc-wf-uncert)');
            const latchHover = document.createElementNS(SVG_NS, 'rect');
            latchHover.setAttribute('x', LABEL_W);
            latchHover.setAttribute('y', rowTop(2));
            latchHover.setAttribute('width', waveW);
            latchHover.setAttribute('height', ROW_H - 8);
            latchHover.setAttribute('fill', 'transparent');
            const latchTitle = document.createElementNS(SVG_NS, 'title');
            latchTitle.textContent = activeIsRising
                ? 'Internal latch state. Transparent (= EN) while CK is '
                  + 'LOW; opaque (held) while CK is HIGH.'
                : 'Internal latch state. Transparent (= EN) while CK is '
                  + 'HIGH; opaque (held) while CK is LOW.';
            latchHover.appendChild(latchTitle);
            svg.appendChild(latchHover);
        }

        // ── GCK row. For latch flavors: GCK = CK ∧ latched(EN).
        // For mux/other: GCK = CK ∧ EN (combinational, glitch-prone).
        const gckIdx = rows.length - 1;
        const gckSegs = [];
        const gateSrc = showLatch ? latchSegs : enSegs;
        const gateLevel = (t) => {
            for (const s of gateSrc) {
                if (t >= s.t0 && t < s.t1) return s.level;
            }
            return 0;
        };
        for (let i = 0; i < cycles; ++i) {
            const tCkLow0 = i * period;
            const tCkHigh = i * period + period / 2;
            const tCkLow1 = (i + 1) * period;
            // CK-low half: GCK forced low regardless of gate.
            gckSegs.push({ t0: tCkLow0, t1: tCkHigh, level: 0 });
            // CK-high half: GCK = gate (latched_EN or raw EN).
            const gateAtHigh = gateLevel(tCkHigh + 0.001);
            gckSegs.push({ t0: tCkHigh, t1: tCkLow1, level: gateAtHigh });
        }
        drawSquare(gckIdx, gckSegs, 'var(--sdc-wf-valid)');
        const gckHover = document.createElementNS(SVG_NS, 'rect');
        gckHover.setAttribute('x', LABEL_W);
        gckHover.setAttribute('y', rowTop(gckIdx));
        gckHover.setAttribute('width', waveW);
        gckHover.setAttribute('height', ROW_H - 8);
        gckHover.setAttribute('fill', 'transparent');
        const gckTitle = document.createElementNS(SVG_NS, 'title');
        gckTitle.textContent = showLatch
            ? 'GCK = CK ∧ latched(EN). Pulses pass when EN was high '
              + 'during the most recent transparent half of CK.'
            : 'GCK = CK ∧ EN. No internal latch — beware glitches if '
              + 'EN transitions during CK\'s active level.';
        gckHover.appendChild(gckTitle);
        svg.appendChild(gckHover);

        return svg;
    }

    // Compact "clock pin" badge row at the top of an expanded instance
    // card — surfaces which pin is the CK/CLK input and which clock domain
    // drives it, without burning a diagram lane on a pin whose timing is
    // already represented by the shared CLK waveform on every sub-card.
    //
    // For latches the "clock" input is really a level-sensitive enable
    // (G/EN/GATE) that gates transparency — naming it "Enable pin"
    // matches how designers think about latches and avoids implying a
    // rising-edge capture model that doesn't apply.
    _renderEndpointClockPinSection(pins, timeUnit, instanceKind,
                                   instancePath) {
        const isLatch = instanceKind === 'latch';
        const sec = document.createElement('div');
        sec.className = 'sdc-ep-clockpin';
        if (isLatch) sec.classList.add('sdc-ep-enablepin');
        sec.style.cssText =
            'border:1px solid var(--border-subtle);border-radius:4px;' +
            'overflow:hidden;background:var(--bg-input);margin-bottom:6px;';
        const hdr = document.createElement('div');
        hdr.style.cssText =
            'padding:4px 8px;background:var(--bg-header);' +
            'border-bottom:1px solid var(--border-subtle);' +
            'font-size:12px;color:var(--fg-primary);font-weight:600;';
        const noun = isLatch ? 'Enable pin' : 'Clock pin';
        hdr.textContent = `${noun}${pins.length === 1 ? '' : 's'}`;
        if (isLatch) {
            hdr.title = 'level-sensitive enable — latch is transparent ' +
                'while this signal is asserted';
        }
        sec.appendChild(hdr);
        const list = document.createElement('div');
        list.style.cssText =
            'padding:4px 8px;display:flex;flex-direction:column;gap:2px;';
        for (const p of pins) {
            const row = document.createElement('div');
            row.style.cssText =
                'font-family:monospace;font-size:12px;color:var(--fg-primary);' +
                'display:flex;align-items:center;gap:6px;flex-wrap:wrap;';
            const nm = document.createElement('span');
            // Show just the pin leaf — the surrounding card already
            // names the instance, so the full path is redundant.
            nm.textContent = this._pinLeafName(p.name, instancePath);
            nm.title = p.name;
            this._linkifyPin(nm, p, 'name');
            row.appendChild(nm);
            const clocks = p.clocks || [];
            if (clocks.length > 0) {
                for (const c of clocks) {
                    const tag = document.createElement('span');
                    tag.style.cssText =
                        'font-size:11px;color:var(--fg-muted);font-weight:400;';
                    const parts = [c.name || '(unnamed)'];
                    if (c.period != null) {
                        parts.push(`T=${c.period.toPrecision(3)}${timeUnit}`);
                        parts.push(this._periodToFreq(c.period));
                    }
                    tag.textContent = '⏱ ' + parts.join('  ');
                    row.appendChild(tag);
                }
            }
            list.appendChild(row);
        }
        sec.appendChild(list);
        return sec;
    }

    // Pins on this instance with no clock domain — render as a compact
    // labelled list rather than an empty "(no clock)" sub-card with a
    // diagram body that has nothing to draw. Common cases:
    //   • macro pins where the constant input has no clock association
    //   • latches whose data inputs are tied to a constant
    //   • async pins on cells without recovery/removal arcs
    // Each pin name is still clickable → forwards to the inspector.
    _renderEndpointNoClockSection(pins, instancePath) {
        const sec = document.createElement('div');
        sec.className = 'sdc-ep-noclock';
        sec.style.cssText =
            'border:1px solid var(--border-subtle);border-radius:4px;' +
            'overflow:hidden;background:var(--bg-input);';
        const hdr = document.createElement('div');
        hdr.style.cssText =
            'padding:4px 8px;background:var(--bg-header);' +
            'border-bottom:1px solid var(--border-subtle);' +
            'font-size:12px;color:var(--fg-primary);font-weight:600;' +
            'display:flex;align-items:center;gap:8px;';
        const lbl = document.createElement('span');
        lbl.textContent = `No clock domain (${pins.length} pin${pins.length === 1 ? '' : 's'})`;
        hdr.appendChild(lbl);
        const sub = document.createElement('span');
        sub.style.cssText =
            'font-size:11px;color:var(--fg-muted);font-weight:400;';
        sub.textContent = '— no setup/hold check available; pins shown for reference';
        hdr.appendChild(sub);
        sec.appendChild(hdr);

        // Cross-reference each pin against set_case_analysis /
        // set_logic_zero/one/dc declarations from the Case Analysis
        // tab — lets us flag pins that are tied to a constant so the
        // user knows there's no real CDC concern there. The lookup is
        // built lazily from the already-loaded _caseAnalysis array.
        const constants = this._buildEndpointConstantLookup();

        // Collapse `name[N]` siblings into one row: a 32-bit memory
        // address bus shouldn't render as 32 individual rows. Bus
        // rows expand on-click to reveal the individual bits, mirroring
        // the bus-expand affordance on the multi-lane waveform.
        // Instance prefix is stripped from labels (solo + bus root) so
        // the rows show just the pin/bus leaf.
        const groups = this._collapseEndpointBusPins(pins, instancePath);

        const list = document.createElement('div');
        list.style.cssText =
            'padding:4px 8px;display:flex;flex-direction:column;gap:1px;';

        const rowStyle =
            'font-family:monospace;font-size:12px;color:var(--fg-primary);' +
            'padding:1px 0;display:flex;align-items:center;gap:6px;';

        // Build one bit-row for an individual pin inside an expanded
        // bus. Same shape as the top-level pin row but indented and
        // with the per-bit constant/title computed from the bit pin.
        const buildBitRow = (pin) => {
            const r = document.createElement('div');
            r.style.cssText = rowStyle + 'padding-left:18px;';
            const n = document.createElement('span');
            n.textContent = this._pinLeafName(pin.name, instancePath);
            n.title = pin.name;
            this._linkifyPin(n, pin, 'name');
            r.appendChild(n);
            const ci = this._collectGroupConstants([pin], constants);
            if (ci) r.appendChild(this._makeConstantBadge(ci));
            return r;
        };

        for (const g of groups) {
            const row = document.createElement('div');
            const isExpandableBus = g.pins.length > 1;
            row.style.cssText = rowStyle
                + (isExpandableBus ? 'cursor:pointer;' : '');
            // ▶/▼ toggle for buses. Solo pins get a width-equivalent
            // spacer so leaf names line up.
            const toggle = document.createElement('span');
            toggle.style.cssText =
                'display:inline-block;width:12px;font-size:11px;'
                + 'color:var(--fg-muted);user-select:none;';
            toggle.textContent = isExpandableBus ? '▶' : '';
            row.appendChild(toggle);
            const nm = document.createElement('span');
            // `g.label` is already leaf-form (set by
            // _collapseEndpointBusPins). `g.title` carries the full
            // path / pin list for hover.
            nm.textContent = g.label;
            nm.title = g.title;
            // Bus rows aren't linkified — clicking a bus would jump to
            // the inspector for some arbitrary "first" bit, which
            // isn't what the user wants. Expanding the bus is the
            // right action; once expanded, each individual bit row is
            // its own clickable target. Solo pins remain linkified.
            if (!isExpandableBus) {
                this._linkifyPin(nm, g.pins[0], 'name');
            }
            row.appendChild(nm);
            // Tag pins tied to a constant. Three flavours from the
            // Case Analysis tab — set_case_analysis 0/1, the assertion
            // form set_logic_zero/one/dc, and the don't-care variant —
            // each gets its own colour-coded badge.
            const constInfo = this._collectGroupConstants(g.pins, constants);
            if (constInfo) {
                row.appendChild(this._makeConstantBadge(constInfo));
            }
            list.appendChild(row);

            if (isExpandableBus) {
                // Container for the per-bit rows; hidden until first
                // expand. Cached so toggling back hides without
                // re-rendering.
                const bitsBox = document.createElement('div');
                bitsBox.style.cssText =
                    'display:none;flex-direction:column;gap:1px;';
                let built = false;
                // Click the entire row (not just the ▶) so the user
                // doesn't have to aim at a 12px arrow. The toggle
                // glyph is just a visual indicator.
                row.addEventListener('click', (e) => {
                    e.stopPropagation();
                    const expanded = bitsBox.style.display !== 'none';
                    if (expanded) {
                        bitsBox.style.display = 'none';
                        toggle.textContent = '▶';
                    } else {
                        if (!built) {
                            for (const pin of g.pins) {
                                bitsBox.appendChild(buildBitRow(pin));
                            }
                            built = true;
                        }
                        bitsBox.style.display = 'flex';
                        toggle.textContent = '▼';
                    }
                });
                list.appendChild(bitsBox);
            }
        }
        sec.appendChild(list);
        return sec;
    }

    // Map pin path → { kind: 'case'|'logic', value: '0'|'1'|...} from
    // the merged _caseAnalysis array (populated by _loadData via
    // sdc_clock_modes). Cached per-load — the lookup is small relative
    // to the pin count we cross-reference against.
    _buildEndpointConstantLookup() {
        const m = new Map();
        for (const e of (this._caseAnalysis || [])) {
            if (e && e.pin) {
                m.set(e.pin, { kind: e.kind || 'case', value: e.value });
            }
        }
        return m;
    }

    // Collapse `name[N]` siblings into a single group; non-bus pins
    // remain their own group. Returns [{ label, title, pins }, ...].
    // Mirrors the bus-collapse logic on the multi-lane diagram but
    // strictly path-shaped (no setup/hold lane signature).
    _collapseEndpointBusPins(pins, instancePath) {
        const busOf = (name) => {
            const m = name && name.match(/^(.*)\[(\d+)\]$/);
            return m ? { root: m[1], index: +m[2] } : null;
        };
        const groups = new Map();
        const order = [];
        for (const p of pins) {
            const b = busOf(p.name);
            const key = b ? `bus|${b.root}` : `solo|${p.name}`;
            if (!groups.has(key)) {
                groups.set(key, { root: b ? b.root : p.name,
                                  isBus: !!b,
                                  pins: [], indices: [] });
                order.push(key);
            }
            const g = groups.get(key);
            g.pins.push(p);
            if (b) g.indices.push(b.index);
        }
        const out = [];
        for (const key of order) {
            const g = groups.get(key);
            if (!g.isBus || g.pins.length <= 1) {
                // Solo / orphan-bus rows: leaf-only label so the
                // surrounding instance card doesn't have its
                // hierarchy duplicated on every line.
                out.push({
                    label: this._pinLeafName(g.pins[0].name, instancePath),
                    title: g.pins[0].name,
                    pins:  g.pins,
                });
                continue;
            }
            const sorted = [...g.indices].sort((a, b) => a - b);
            const min = sorted[0];
            const max = sorted[sorted.length - 1];
            const contiguous = sorted.length === (max - min + 1);
            // Bus root is also trimmed to its leaf — the bus name is
            // unique within the instance so the prefix is redundant.
            const rootLeaf = this._pinLeafName(g.root, instancePath);
            const label = contiguous
                ? `${rootLeaf}[${max}:${min}]  (${g.pins.length} pins)`
                : `${rootLeaf}  (${g.pins.length} pins)`;
            const sortedPins = [...g.pins].sort((a, b) => {
                const ai = busOf(a.name).index;
                const bi = busOf(b.name).index;
                return bi - ai;  // hi-to-lo to match the label
            });
            out.push({
                label,
                title: sortedPins.map(p => p.name).join('\n'),
                pins:  sortedPins,
            });
        }
        return out;
    }

    // Walk a group's pins through the constant-lookup table. Returns
    // `null` when no pin in the group is tied to a constant; otherwise
    // `{value, kind, n_total, n_const, mixed}` summarising what
    // fraction of the group is constant and whether all entries agree.
    _collectGroupConstants(pins, lookup) {
        if (!lookup || lookup.size === 0) return null;
        let firstValue = null;
        let firstKind  = null;
        let mixed = false;
        let nConst = 0;
        for (const p of pins) {
            const c = lookup.get(p.name);
            if (!c) continue;
            ++nConst;
            if (firstValue === null) {
                firstValue = c.value;
                firstKind  = c.kind;
            } else if (c.value !== firstValue) {
                mixed = true;
            }
        }
        if (nConst === 0) return null;
        return {
            value: mixed ? '?' : firstValue,
            kind:  firstKind,
            n_total: pins.length,
            n_const: nConst,
            mixed,
        };
    }

    _makeConstantBadge(info) {
        const badge = document.createElement('span');
        // Colour by value: 0 = muted blue, 1 = muted yellow, x/dc = grey.
        // Same idea as the Case Analysis tab strip so users connect
        // the pin-level mark here to the source declaration there.
        const bgByVal = {
            '0':    'rgba(80, 140, 220, 0.20)',
            '1':    'rgba(220, 180, 60, 0.20)',
            'rise': 'rgba(220, 180, 60, 0.20)',
            'fall': 'rgba(80, 140, 220, 0.20)',
        };
        const bg = bgByVal[info.value] || 'rgba(150, 150, 150, 0.20)';
        badge.style.cssText =
            `font-size:11px;padding:0 5px;border-radius:8px;`
            + `background:${bg};color:var(--fg-primary);`
            + `font-weight:600;font-family:monospace;`;
        const label = info.value === '?' ? 'mixed const' : `≡ ${info.value}`;
        if (info.n_total > 1 && !info.mixed
                && info.n_const === info.n_total) {
            badge.textContent = `${label}  (all ${info.n_total})`;
        } else if (info.n_total > 1) {
            badge.textContent = `${label}  (${info.n_const} of ${info.n_total})`;
        } else {
            badge.textContent = label;
        }
        const cmd = info.kind === 'logic' ? 'set_logic_*' : 'set_case_analysis';
        badge.title = info.mixed
            ? `bus pins are tied to mixed constants via ${cmd} — `
              + `inspect individually for the per-bit values`
            : `pin tied to constant ${info.value} via ${cmd} — `
              + `the path through this pin won't toggle, so STA's "no clock `
              + `domain" status reflects the constant tie, not missing SDC`;
        return badge;
    }

    // Dedupe applicable exceptions across every pin in the card; one row
    // per unique exception (keyed by its numeric id), with an "applies
    // to: D, SET_n" tag when the exception scope is narrower than the
    // full pin set on the card.
    _renderEndpointInstanceExceptions(pins, timeUnit) {
        const byId = new Map();  // id → { exc, pinNames: Set }
        for (const p of pins) {
            for (const exc of (p.exceptions || [])) {
                const id = (exc && exc.id != null) ? exc.id : JSON.stringify(exc);
                if (!byId.has(id)) byId.set(id, { exc, pinNames: new Set() });
                byId.get(id).pinNames.add(p.name);
            }
        }
        if (byId.size === 0) return null;
        const allPinNames = new Set(pins.map(p => p.name));
        const sec = document.createElement('div');
        sec.className = 'sdc-ep-card-exceptions';
        sec.style.cssText =
            'margin-top:6px;border:1px solid var(--border-subtle);border-radius:4px;' +
            'overflow:hidden;background:var(--bg-input);';
        const hdr = document.createElement('div');
        hdr.style.cssText =
            'padding:3px 8px;background:var(--bg-header);' +
            'border-bottom:1px solid var(--border-subtle);' +
            'font-size:12px;font-weight:600;color:var(--fg-primary);';
        hdr.textContent = `Applicable Exceptions (${byId.size})`;
        sec.appendChild(hdr);
        const body = document.createElement('div');
        body.style.cssText = 'padding:4px 8px;display:flex;flex-direction:column;gap:3px;';
        for (const { exc, pinNames } of byId.values()) {
            const row = this._makeExcRow(exc, timeUnit);
            // If the exception only hits a subset of this card's pins,
            // tag it with "applies to: …" so the user knows it's not
            // universal across the instance.
            if (pinNames.size < allPinNames.size) {
                const tag = document.createElement('div');
                tag.style.cssText =
                    'padding:1px 8px;font-size:11px;color:var(--fg-muted);' +
                    'font-style:italic;';
                tag.textContent =
                    `applies to: ${[...pinNames]
                        .map(n => n.split('/').pop())
                        .join(', ')}`;
                row.appendChild(tag);
            }
            body.appendChild(row);
        }
        sec.appendChild(body);
        return sec;
    }

    _showEpListMoreFooter(text) {
        let f = this._epListArea.querySelector('.sdc-ep-list-footer');
        if (!f) {
            f = document.createElement('div');
            f.className = 'sdc-ep-list-footer';
            f.style.cssText =
                'padding:8px;text-align:center;font-size:12px;' +
                'color:var(--fg-muted);font-style:italic;';
            this._epListArea.appendChild(f);
        }
        f.textContent = text;
    }

    _installEpListInfiniteScroll() {
        const handler = () => {
            const el = this._epListArea;
            if (!el) return;
            if (this._epListLoading) return;
            if (!this._epListAll) return;
            if (this._epListAll.length >= (this._epListTotal || 0)) return;
            const remaining = el.scrollHeight - el.scrollTop - el.clientHeight;
            if (remaining < el.clientHeight * 1.5) {
                this._loadEndpointList({});
            }
        };
        this._epListScrollHandler = handler;
        this._epListArea.addEventListener('scroll', handler);
    }

    // After a batch lands, fetch more if the scroll area isn't yet
    // scrollable. Without this the user sees one or two cards filling
    // less than the viewport, no scrollbar, and no way to advance the
    // pagination — the scroll handler never fires because there's
    // nothing to scroll. Defer one frame so the freshly-appended
    // cards have laid out before we measure scrollHeight.
    _maybeTopUpEpList() {
        if (typeof requestAnimationFrame === 'undefined') return;
        requestAnimationFrame(() => {
            const el = this._epListArea;
            if (!el) return;
            if (this._epListLoading) return;
            if (!this._epListAll) return;
            if (this._epListAll.length >= (this._epListTotal || 0)) return;
            // Trigger a fetch when the content can't be scrolled past
            // the visible window — i.e. effectively no scrollbar, or
            // the bottom is already in view.
            if (el.scrollHeight <= el.clientHeight + 16) {
                this._loadEndpointList({});
            }
        });
    }

    async _queryEndpoint(pinName) {
        if (!pinName) return;
        // Reset pagination state for the new query and stamp a fresh token
        // so any in-flight fetches from the previous query are dropped.
        this._epQueryToken = (this._epQueryToken || 0) + 1;
        const token = this._epQueryToken;
        this._epPinName    = pinName;
        this._epAllPins    = [];
        this._epTotal      = 0;
        this._epFetchingMore = false;
        this._epResultArea.innerHTML =
            '<div style="padding:12px;color:var(--fg-muted);font-style:italic;">Querying…</div>';
        try {
            await this._app.websocketManager.readyPromise;
            const data = await this._requestWithTimeout({
                type: 'sdc_endpoint',
                pin: pinName,
                offset: 0,
                limit: this._batchSize(),
            });
            // Stale token? user issued a new query while this was in flight.
            if (token !== this._epQueryToken) return;
            this._epAllPins = data.pins || [];
            this._epTotal   = (typeof data.total === 'number')
                ? data.total : this._epAllPins.length;
            this._renderEndpointResult(pinName, data);
            this._installEpInfiniteScroll();
        } catch (e) {
            if (token !== this._epQueryToken) return;
            console.error('[SDC] endpoint query error:', e);
            const msg = e && e.message ? e.message : String(e);
            this._epResultArea.innerHTML =
                `<div style="padding:12px;color:var(--fg-muted);font-style:italic;">Error: ${msg}</div>`;
        }
    }

    _installEpInfiniteScroll() {
        if (this._epScrollHandler) return;
        const handler = () => {
            const el = this._epResultArea;
            if (!el) return;
            if (this._epFetchingMore) return;
            if (!this._epAllPins) return;
            if (this._epAllPins.length >= this._epTotal) return;
            const slack = el.clientHeight * 1.5;
            if (el.scrollTop + el.clientHeight + slack >= el.scrollHeight) {
                this._fetchEpMore();
            }
        };
        this._epResultArea.addEventListener('scroll', handler);
        this._epScrollHandler = handler;
    }

    async _fetchEpMore() {
        if (this._epFetchingMore) return;
        if (!this._epAllPins) return;
        if (this._epAllPins.length >= this._epTotal) return;
        const token = this._epQueryToken;
        this._epFetchingMore = true;
        this._showEpMoreFooter('Loading more…');
        try {
            const data = await this._requestWithTimeout({
                type: 'sdc_endpoint',
                pin:    this._epPinName,
                offset: this._epAllPins.length,
                limit:  this._batchSize(),
            });
            if (token !== this._epQueryToken) return;  // user moved on
            const more = data.pins || [];
            this._epAllPins = this._epAllPins.concat(more);
            if (typeof data.total === 'number') this._epTotal = data.total;
            // Re-render with the merged list. _renderEndpointResult clears
            // innerHTML, which collapses the scroll content and forces
            // scrollTop to 0 mid-scroll. Capture and restore the position.
            const savedScrollTop = this._epResultArea.scrollTop;
            this._renderEndpointResult(this._epPinName, {
                ...data,
                pins:  this._epAllPins,
                total: this._epTotal,
            });
            this._epResultArea.scrollTop = savedScrollTop;
        } catch (e) {
            if (token !== this._epQueryToken) return;
            console.warn('[SDC] endpoint paginated fetch failed', e);
            this._showEpMoreFooter(
                `Failed to load more: ${(e && e.message) || e}`);
        } finally {
            this._epFetchingMore = false;
        }
    }

    _showEpMoreFooter(text) {
        let f = this._epResultArea.querySelector('.sdc-ep-more-footer');
        if (!f) {
            f = document.createElement('div');
            f.className = 'sdc-ep-more-footer';
            f.style.cssText =
                'padding:10px;text-align:center;font-size:12px;' +
                'color:var(--fg-muted);font-style:italic;';
            this._epResultArea.appendChild(f);
        }
        f.textContent = text;
    }

    _renderEndpointResult(pinName, data) {
        this._epResultArea.innerHTML = '';

        if (!data.found) {
            this._epResultArea.innerHTML =
                `<div style="padding:12px;color:var(--fg-muted);">` +
                `No pin found matching: <span style="font-family:monospace;">${pinName}</span></div>`;
            return;
        }

        const timeUnit = data.time_unit || 'ns';
        const pins = data.pins || [];

        // Render every pin returned by the backend. The previous filter
        // here only kept pins with explicit setup/hold library checks (or
        // exceptions / port_delays); that silently dropped legitimate
        // endpoints — async preset/clear pins (recovery/removal arcs),
        // combinational endpoints reached through set_max_delay -to,
        // ports with neither set_input_delay nor an exception attached,
        // etc. — even though sta::Search::endpoints() considers them
        // real endpoints. _renderPinCard already gracefully handles
        // empty clocks/exceptions/port_delays sections, so trusting STA's
        // endpoint set gives the user a card per actual endpoint instead
        // of a "No timing endpoints found" message that's wrong.
        if (pins.length === 0) {
            const none = document.createElement('div');
            none.style.cssText = 'padding:12px;color:var(--fg-muted);font-style:italic;';
            none.textContent = `No pin found matching "${pinName}".`;
            this._epResultArea.appendChild(none);
            return;
        }

        const totalLoaded = pins.length;
        const totalKnown  = (typeof data.total === 'number')
            ? data.total : totalLoaded;
        if (data.multi || totalKnown > totalLoaded) {
            const mhdr = document.createElement('div');
            mhdr.style.cssText =
                'padding:4px 10px;margin-bottom:6px;font-size:12px;color:var(--fg-muted);' +
                'border:1px solid var(--border);border-radius:4px;background:var(--bg-header);';
            const matchSuffix = totalKnown > totalLoaded
                ? ` (loaded ${totalLoaded} of ${totalKnown}, scroll for more)`
                : '';
            mhdr.textContent =
                `${pins.length} endpoint${pins.length !== 1 ? 's' : ''} ` +
                `matched "${pinName}"${matchSuffix}`;
            this._epResultArea.appendChild(mhdr);
        }

        for (const pinData of pins) {
            this._renderPinCard(pinData, timeUnit);
        }

        // Pagination footer for the endpoint scroll area.  Mirrors the
        // port-delays footer: shows "loaded N of M" when more is on the way,
        // or "all N loaded" once everything is in.
        if (totalKnown > totalLoaded) {
            this._showEpMoreFooter(
                `Loaded ${totalLoaded} of ${totalKnown} matches — scroll for more`);
        } else if (totalKnown > this._batchSize()) {
            this._showEpMoreFooter(`All ${totalKnown} matches loaded`);
        }
    }

    _renderPinCard(pinData, timeUnit) {
        // Outer card wrapping pin header + all nested sections
        const card = document.createElement('div');
        card.style.cssText =
            'margin-bottom:10px;border:1px solid var(--border);border-radius:4px;overflow:hidden;';

        // Pin header
        const pinHeader = document.createElement('div');
        pinHeader.style.cssText =
            'padding:5px 10px;background:var(--bg-header);border-bottom:1px solid var(--border);' +
            'display:flex;align-items:center;gap:8px;font-size:12px;';
        const pinIcon = document.createElement('span');
        pinIcon.style.cssText =
            'font-size:11px;padding:1px 5px;border-radius:3px;font-weight:600;' +
            'background:var(--bg-input);color:var(--fg-muted);';
        pinIcon.textContent = pinData.is_port ? 'PORT' : 'PIN';
        const pinNameEl = document.createElement('span');
        pinNameEl.style.cssText =
            'font-family:monospace;font-weight:600;color:var(--fg-primary);'
            + 'flex:1;min-width:0;' + TRUNCATE_PATH_CSS;
        pinNameEl.textContent = pinData.name;
        // Always set the tooltip — even if the path doesn't currently
        // truncate, narrower windows or longer hierarchies will start
        // clipping it, and the user can hover to see the full text.
        pinNameEl.title = pinData.name;
        // Endpoint result pins carry `name_odb_type` + `name_odb_id` fields.
        this._linkifyPin(pinNameEl, pinData, 'name');
        pinHeader.appendChild(pinIcon);
        pinHeader.appendChild(pinNameEl);
        card.appendChild(pinHeader);

        // Card body — nested sections live here
        const body = document.createElement('div');
        body.style.cssText =
            'padding:6px 8px;display:flex;flex-direction:column;gap:6px;background:var(--bg-main);';

        // ── Clock domains (internal flip-flop pins only) ─────────────────────
        if (!pinData.is_port) {
            const clocks = pinData.clocks || [];
            body.appendChild(this._makeEpSection(
                `Clock Domains (${clocks.length})`,
                (b) => {
                    if (clocks.length === 0) {
                        const empty = document.createElement('div');
                        empty.style.cssText =
                            'padding:8px 10px;font-size:12px;color:var(--fg-muted);font-style:italic;';
                        empty.textContent = 'No clock domain found for this pin.';
                        b.appendChild(empty);
                    } else {
                        b.style.cssText = 'padding:8px;display:flex;flex-direction:column;gap:8px;';
                        for (const clk of clocks) {
                            b.appendChild(this._renderClockDiagramCard(clk, timeUnit));
                        }
                    }
                }
            ));
        }

        // ── Applicable exceptions ────────────────────────────────────────────
        const exceptions = pinData.exceptions || [];
        body.appendChild(this._makeEpSection(
            `Applicable Exceptions (${exceptions.length})`,
            (b) => {
                if (exceptions.length === 0) {
                    const empty = document.createElement('div');
                    empty.style.cssText =
                        'padding:8px 10px;font-size:12px;color:var(--fg-muted);font-style:italic;';
                    empty.textContent = 'No timing exceptions apply to this pin.';
                    b.appendChild(empty);
                } else {
                    b.style.padding = '6px 8px';
                    b.style.display = 'flex';
                    b.style.flexDirection = 'column';
                    b.style.gap = '4px';
                    for (const exc of exceptions) {
                        b.appendChild(this._makeExcRow(exc, timeUnit));
                    }
                }
            }
        ));

        // ── Port delays (top-level ports only) ───────────────────────────────
        const portDelays = pinData.port_delays || [];
        if (portDelays.length > 0) {
            body.appendChild(this._makeEpSection(
                `Port Delays (${portDelays.length})`,
                (b) => {
                    for (const entry of portDelays) {
                        const diagram = this._renderPortDelayDiagram(entry, timeUnit);
                        if (diagram) b.appendChild(diagram);
                    }
                }
            ));
        }

        card.appendChild(body);
        this._epResultArea.appendChild(card);
    }

    // Compact timing-diagram card for a clock domain in the Endpoint tab.
    // Left column: clock name + total setup/hold. Right: CLK waveform + DATA bar + legend.
    _renderClockDiagramCard(clk, timeUnit) {
        const card = document.createElement('div');
        card.style.cssText =
            'border:1px solid var(--border);border-radius:4px;overflow:hidden;';

        // Header: clock name only — timing params are in the left label column of the SVG
        const hdr = document.createElement('div');
        hdr.style.cssText =
            'padding:3px 8px;background:var(--bg-header);border-bottom:1px solid var(--border-subtle);' +
            'font-family:monospace;font-size:12px;font-weight:600;color:var(--fg-primary);';
        hdr.textContent = clk.name;
        card.appendChild(hdr);

        const T = clk.period;
        const waveform = clk.waveform;
        if (!waveform || waveform.length < 2 || !T) {
            const noWave = document.createElement('div');
            noWave.style.cssText =
                'padding:6px 8px;font-size:12px;color:var(--fg-muted);font-style:italic;';
            noWave.textContent = 'No waveform data.';
            card.appendChild(noWave);
            return card;
        }

        const su  = clk.uncertainty_setup  || 0;
        const hu  = clk.uncertainty_hold   || 0;
        const tsu = clk.library_setup      || 0;
        const th  = clk.library_hold       || 0;

        // Layout: left label column (LABEL_W) + waveform area (WAVE_W).
        // Deduct ~52px for nested card borders + padding in the endpoint tab.
        const LABEL_W = 130;
        const totalW  = Math.max(360, (this._epResultArea.clientWidth || 500) - 52);
        const WAVE_W  = totalW - LABEL_W;
        const { LEG_Y, CLK_HIGH, CLK_LOW, CLK_MID } = DIAGRAM_CONST;
        const DATA_TOP = 50, DATA_BOT = 72;
        const AXIS_Y   = 82;
        const TICK_ROW = [AXIS_Y + 11, AXIS_Y + 23];
        const SVG_H    = 120;  // tall enough for 6-line left label column

        const tRef   = waveform[0];
        const thAbs  = Math.abs(th);  // always positive for bar drawing
        const POST   = Math.max((thAbs + hu) * 2.5, T * 0.25);
        const tStart = -T;
        const tEnd   = POST;
        const tRange = tEnd - tStart;
        const tx = (t) => LABEL_W + ((t - tStart) / tRange) * WAVE_W;

        const { edges: allEdges, prevY }
            = this._collectClockEdges(waveform, T, tRef, tStart, tEnd);

        const svg = this._makeSvg(totalW, SVG_H);

        // ── Left label column ─────────────────────────────────────────────────
        // Bar visual order: valid | tsu | Su | edge | Hu | |th| | valid
        // Su/Hu are adjacent to the edge (inner); tsu/|th| are outer.
        // th is shown as |th| so both hold components appear positive.
        const tSetupDeadline = -(tsu + su);      // leftmost setup boundary
        const tHoldVisual    =  hu + thAbs;      // rightmost hold boundary (using |th|)
        const totalSetup = tsu + su;
        const totalHold  = tHoldVisual;

        this._svgText(svg, 4, CLK_MID + 4, 'CLK', 'var(--fg-muted)', 8, 'start');

        // Setup breakdown (outer tsu → inner setup-unc → edge).
        // Each label carries a native SVG <title> tooltip spelling out the
        // full meaning so users don't have to decode the short names.
        const totalSetupText = this._svgText(svg, 4, DATA_TOP + 7,
            `setup: ${totalSetup.toPrecision(3)}${timeUnit}`,
            'var(--sdc-text-setup)', 8, 'start');
        this._svgTitle(totalSetupText,
            'total setup margin = library setup + setup clock uncertainty');
        if (tsu !== 0) {
            const el = this._svgText(svg, 10, DATA_TOP + 16,
                `${TIMING_LABELS.tsu.short} = ${tsu.toPrecision(3)}${timeUnit}`,
                'var(--sdc-text-setup)', 7, 'start');
            this._svgTitle(el, TIMING_LABELS.tsu.tip);
        }
        if (su !== 0) {
            const el = this._svgText(svg, 10, DATA_TOP + 24,
                `${TIMING_LABELS.setup_unc.short} = ${su.toPrecision(3)}${timeUnit}`,
                'var(--sdc-wf-setup-unc)', 7, 'start');
            this._svgTitle(el, TIMING_LABELS.setup_unc.tip);
        }

        // Hold breakdown (edge → inner hold-unc → outer |th|)
        const totalHoldText = this._svgText(svg, 4, DATA_TOP + 35,
            `hold: ${totalHold.toPrecision(3)}${timeUnit}`,
            'var(--sdc-text-hold)', 8, 'start');
        this._svgTitle(totalHoldText,
            'total hold margin = library hold + hold clock uncertainty');
        if (hu !== 0) {
            const el = this._svgText(svg, 10, DATA_TOP + 44,
                `${TIMING_LABELS.hold_unc.short} = ${hu.toPrecision(3)}${timeUnit}`,
                'var(--sdc-wf-hold-unc)', 7, 'start');
            this._svgTitle(el, TIMING_LABELS.hold_unc.tip);
        }
        if (thAbs !== 0) {
            const el = this._svgText(svg, 10, DATA_TOP + 52,
                `${TIMING_LABELS.th.short} = ${thAbs.toPrecision(3)}${timeUnit}`,
                'var(--sdc-text-hold)', 7, 'start');
            this._svgTitle(el, TIMING_LABELS.th.tip);
        }

        // ── Clock waveform path ───────────────────────────────────────────────
        this._drawClockPath(svg, allEdges, tx, CLK_HIGH, CLK_LOW, prevY,
                            tStart, tEnd, 'var(--canvas-axis)', '1.5');

        // Dashed reference line at t=0 (capture edge).
        const refLine = this._svgLine(svg, tx(0), CLK_HIGH, tx(0), AXIS_Y,
                                      'var(--border-subtle)', '1');
        refLine.setAttribute('stroke-dasharray', '3,3');
        refLine.dataset.ref = 'capture';

        // ── Data timing bar ───────────────────────────────────────────────────
        // Bands paint via _svgBandRect (kind-driven). 0.85 opacity matches
        // the port-delay diagrams and lets overlay axis lines peek through.
        // COL_* aliases stay around because the legend strip below renders
        // swatches by CSS color value.
        const COL_VALID      = BAND_COLORS['valid'];
        const COL_SETUP_UNC  = BAND_COLORS['setup-unc'];
        const COL_HOLD_UNC   = BAND_COLORS['hold-unc'];
        const COL_LIB_SETUP  = BAND_COLORS['lib-setup'];
        const COL_LIB_HOLD   = BAND_COLORS['lib-hold'];
        const BAR_OPA = DIAGRAM_CONST.BAR_OPACITY;
        const drawBar = (t1, t2, kind) => {
            const x1 = Math.max(LABEL_W, tx(t1));
            const x2 = Math.min(totalW, tx(t2));
            return this._svgBandRect(svg, x1, DATA_TOP, x2 - x1,
                                     DATA_BOT - DATA_TOP, kind, BAR_OPA);
        };

        // Visual order: valid | tsu | Su | edge | Hu | |th| | valid
        drawBar(tStart, tSetupDeadline, 'valid');            // valid (left)
        if (tsu > 0) drawBar(tSetupDeadline, -su, 'lib-setup'); // tsu (outer setup)
        if (su > 0)  drawBar(-su, 0, 'setup-unc');              // Su  (inner setup)
        if (hu > 0)  drawBar(0, hu, 'hold-unc');                // Hu  (inner hold)
        if (thAbs > 0) drawBar(hu, tHoldVisual, 'lib-hold');    // |th| (outer hold)
        drawBar(tHoldVisual, tEnd, 'valid');                 // valid (right)

        // Bar outline.
        this._svgOutlineRect(svg, LABEL_W, DATA_TOP,
                             WAVE_W, DATA_BOT - DATA_TOP);

        // ── Axis — two-row ticks ──────────────────────────────────────────────
        this._svgLine(svg, LABEL_W, AXIS_Y, totalW, AXIS_Y, 'var(--canvas-axis)', 1);

        const clkTicks = [];
        const addClkTick = (t, label, row) => {
            const x = Math.max(LABEL_W + 3, Math.min(totalW - 3, tx(t)));
            clkTicks.push({ x, label, row });
        };

        // Row 0: outer boundaries (setup deadline and hold visual boundary)
        if (tsu > 0 || su > 0) addClkTick(tSetupDeadline, `${tSetupDeadline.toPrecision(3)}`, 0);
        if (thAbs > 0 || hu > 0) addClkTick(tHoldVisual, `${tHoldVisual.toPrecision(3)}`, 0);
        // Row 1: launch edge (shows period) and capture edge
        addClkTick(tStart, `T=${T.toPrecision(4)}${timeUnit}`, 1);
        addClkTick(0, '0', 1);

        for (const { x, label, row } of clkTicks) {
            this._svgLine(svg, x, AXIS_Y - 2, x, AXIS_Y + 3, 'var(--canvas-axis)', 1);
            const ty   = TICK_ROW[row];
            const half = label.length * 2.5;
            const lx2  = Math.max(LABEL_W + half + 1, Math.min(totalW - half - 1, x));
            this._svgText(svg, lx2, ty, label, 'var(--canvas-label)', 8, 'middle');
        }

        // ── Legend (top strip, short names — values are in left column) ────────
        // Rendered left-to-right so it can never overflow the right edge.
        // Legend order matches bar order: valid | tsu | setup unc | hold unc | th | valid.
        // Every swatch + label has a hover tooltip spelling out the full meaning.
        const validTip = 'data valid — window where the launch-edge data is stable';
        const clkLegItems = [
            { color: COL_VALID,     label: 'valid', tip: validTip },
            ...(tsu > 0   ? [{ color: COL_LIB_SETUP,
                               label: TIMING_LABELS.tsu.short,
                               tip: TIMING_LABELS.tsu.tip }] : []),
            ...(su > 0    ? [{ color: COL_SETUP_UNC,
                               label: TIMING_LABELS.setup_unc.short,
                               tip: TIMING_LABELS.setup_unc.tip }] : []),
            ...(hu > 0    ? [{ color: COL_HOLD_UNC,
                               label: TIMING_LABELS.hold_unc.short,
                               tip: TIMING_LABELS.hold_unc.tip }] : []),
            ...(thAbs > 0 ? [{ color: COL_LIB_HOLD,
                               label: TIMING_LABELS.th.short,
                               tip: TIMING_LABELS.th.tip }] : []),
        ];
        const clkLegItemW = (label) => label.length * 5.5 + 16;
        let lx = LABEL_W + 4;
        for (const { color, label, tip } of clkLegItems) {
            const tw = clkLegItemW(label);
            if (lx + tw > totalW - 4) break;
            const swatch = document.createElementNS(SVG_NS, 'rect');
            swatch.setAttribute('x', lx); swatch.setAttribute('y', LEG_Y - 7);
            swatch.setAttribute('width', 8); swatch.setAttribute('height', 8);
            swatch.style.fill = color; swatch.style.fillOpacity = BAR_OPA;
            this._svgTitle(swatch, tip);
            svg.appendChild(swatch);
            const textEl = this._svgText(svg, lx + 10, LEG_Y, label,
                                         'var(--canvas-label)', 8, 'start');
            this._svgTitle(textEl, tip);
            lx += tw;
        }

        card.appendChild(svg);
        return card;
    }

    // Multi-lane endpoint setup/hold diagram. One shared CLK waveform
    // at the top, one stacked data lane per endpoint pin (or per
    // bus-collapsed group of identical-constraint pins), and one shared
    // axis at the bottom. Each lane paints the same band layout the
    // single-pin _renderClockDiagramCard uses
    //   (valid | tsu | Su | edge | Hu | |th| | valid)
    // but anchored to the SAME time axis across all lanes so the
    // capture edge lines up vertically.
    //
    // Bus collapsing: pins with identical (tsu, th, su, hu) AND a
    // bus-style suffix `name[N]` collapse into one lane labelled
    // `name[hi:lo]` (or `name[N pins]` for non-contiguous groups). The
    // collapsed lane's <title> spells out the full pin list so the
    // user can hover to see exactly which pins are folded in.
    _renderEndpointMultiLaneDiagram(clkInfo, pins, timeUnit, opts) {
        opts = opts || {};
        const expandedBuses = opts.expandedBuses instanceof Set
            ? opts.expandedBuses : new Set();
        const onBusExpand = typeof opts.onBusExpand === 'function'
            ? opts.onBusExpand : null;
        // For latches t=0 is the closing edge of the enable (when D is
        // captured), not the rising edge. The closing edge is whichever
        // transition the setup-check arc references — backend emits this
        // as `capture_edge: 'rise' | 'fall'` per pin:
        //   • active-high latch → close on fall (capture_edge = 'fall')
        //   • active-low  latch → close on rise (capture_edge = 'rise')
        // We pick the first input pin with a known capture_edge as the
        // sub-card anchor; pins without one (e.g. async clears) fall
        // back to whichever pin set the anchor.
        const isLatch = opts.instanceKind === 'latch';
        let latchCloseEdge = null;  // 'rise' | 'fall' | null
        if (isLatch) {
            for (const p of pins) {
                if (p && (p.capture_edge === 'rise' || p.capture_edge === 'fall')) {
                    latchCloseEdge = p.capture_edge;
                    break;
                }
            }
            // Default to fall if no pin reported one (active-high latch
            // is the dominant case in standard libraries).
            if (!latchCloseEdge) latchCloseEdge = 'fall';
        }
        const T = clkInfo.period;
        const waveform = clkInfo.waveform;
        if (!waveform || waveform.length < 2 || !T) {
            const noWave = document.createElement('div');
            noWave.style.cssText =
                'padding:6px 8px;font-size:12px;color:var(--fg-muted);font-style:italic;';
            noWave.textContent = 'No waveform data.';
            return noWave;
        }

        // Collapse bus pins; one lane per result entry below.
        const lanes = this._buildEndpointLanes(
            pins, clkInfo, expandedBuses, opts.instancePath);
        if (lanes.length === 0) return null;

        const LABEL_W = 200;
        const totalW = Math.max(
            420,
            ((this._epListArea && this._epListArea.clientWidth) || 600) - 40);
        const WAVE_W = totalW - LABEL_W;
        const { LEG_Y, CLK_HIGH, CLK_LOW, CLK_MID, LANE_GAP } = DIAGRAM_CONST;
        const LANE_TOP_BASE = 50;
        const LANE_H = 22;
        const lanesEnd = LANE_TOP_BASE + lanes.length * (LANE_H + LANE_GAP);
        const AXIS_Y = lanesEnd + 8;
        const TICK_ROW = [AXIS_Y + 11, AXIS_Y + 23];
        const SVG_H = AXIS_Y + 30;

        // Shared time axis: widest setup margin to the left, widest hold
        // margin to the right; T off the left for the previous launch
        // edge (so the period is visible). All lane bands sit inside
        // [tStart, tEnd] anchored to the capture edge at t=0.
        // Output lanes additionally extend the right side of the axis
        // by their clk-to-Q upper bound so the transition window fits.
        // For latch sub-cards the capture edge is the closing edge of
        // the enable: the falling edge for active-high latches (close =
        // waveform[1]), the rising edge for active-low (close =
        // waveform[0]). Open edge is the other entry. The waveform
        // array uses [rise, fall] ordering by convention.
        let tRef = waveform[0];
        let latchOpenT = 0;
        if (isLatch) {
            const closeIdx = (latchCloseEdge === 'rise') ? 0 : 1;
            const openIdx  = 1 - closeIdx;
            tRef = waveform[closeIdx];
            // Express the open edge in the post-shift axis where t=0
            // is the close edge. Active-high → openT = -T/2, active-low
            // → openT = -T/2 too on a symmetric clock; the exact value
            // is whatever the waveform reports.
            latchOpenT = waveform[openIdx] - tRef;
            // Wrap into [-T, 0] so we always show the most-recent
            // transparent window (active-low's open-after-close maps
            // back into the previous period).
            if (latchOpenT > 0) latchOpenT -= T;
        }
        const inputLanes = lanes.filter(L => L.kind !== 'output');
        const outputLanes = lanes.filter(L => L.kind === 'output');
        const maxTsuSu = inputLanes.reduce(
            (m, L) => Math.max(m, L.tsu + L.su), 0);
        const maxThHu = inputLanes.reduce(
            (m, L) => Math.max(m, Math.abs(L.th) + L.hu), 0);
        const maxClkqMax = outputLanes.reduce(
            (m, L) => Math.max(m, L.clkqMax || 0), 0);
        // Time-borrow extension on latch input lanes: explicit limit wins;
        // otherwise the implicit default is the transparent-period width.
        // We resolve it once per lane here so axis sizing and band drawing
        // agree.
        const transparentSpan = isLatch ? Math.max(0, -latchOpenT) : 0;
        const laneEffectiveBorrow = (L) => {
            if (!isLatch || L.kind === 'output') return 0;
            if (L.timeBorrowLimit != null) return L.timeBorrowLimit;
            return transparentSpan;
        };
        const maxBorrow = inputLanes.reduce(
            (m, L) => Math.max(m, laneEffectiveBorrow(L)), 0);
        const POST = Math.max(maxThHu * 2.5, maxClkqMax * 1.15,
                              maxBorrow * 1.1, T * 0.25);
        const tStart = -T;
        const tEnd = POST;
        const tRange = tEnd - tStart;
        const tx = (t) => LABEL_W + ((t - tStart) / tRange) * WAVE_W;
        void maxTsuSu;  // (left margin reference; tx covers it)

        const { edges: allEdges, prevY }
            = this._collectClockEdges(waveform, T, tRef, tStart, tEnd);

        const svg = this._makeSvg(totalW, SVG_H);
        // Tag with a class so tests / styles can find the diagram easily.
        svg.classList.add('sdc-ep-multilane');
        if (isLatch) svg.classList.add('sdc-ep-latch');

        // ── Shared CLK waveform + capture-edge guide ────────────────────
        // Label the row 'CLK' on flops, 'EN' on latches — for a latch
        // this signal is the level-sensitive enable, not a true clock.
        const clkLabel = this._svgText(svg, 4, CLK_MID + 4,
            isLatch ? 'EN' : 'CLK', 'var(--fg-muted)', 8, 'start');
        this._svgTitle(clkLabel, isLatch
            ? `${clkInfo.name} — enable signal (level-sensitive). The latch ` +
              `is transparent while this signal is asserted; D is captured ` +
              `at the closing edge.`
            : `${clkInfo.name} — capturing clock for this domain. ` +
              `The dashed guide marks the capture edge at t = 0.`);
        this._drawClockPath(svg, allEdges, tx, CLK_HIGH, CLK_LOW, prevY,
                            tStart, tEnd, 'var(--canvas-axis)', '1.5');
        const refLine = this._svgLine(svg, tx(0), CLK_HIGH, tx(0), AXIS_Y,
                                      'var(--border-subtle)', '1');
        refLine.setAttribute('stroke-dasharray', '3,3');
        refLine.dataset.ref = 'capture';
        this._svgTitle(refLine, isLatch
            ? 'enable closing edge (t = 0) — D is captured here'
            : 'capture edge (t = 0) — the launching/capturing clock edge ' +
              'this lane is timed against');
        // Latch open edge: dashed guide marking when the latch became
        // transparent. Drawn alongside the existing close-edge guide so
        // the user sees the full open→close span at a glance.
        if (isLatch && latchOpenT < 0 && latchOpenT > tStart) {
            const openLine = this._svgLine(svg, tx(latchOpenT), CLK_HIGH,
                                           tx(latchOpenT), AXIS_Y,
                                           'var(--border-subtle)', '1');
            openLine.setAttribute('stroke-dasharray', '3,3');
            openLine.dataset.ref = 'latch-open';
            this._svgTitle(openLine,
                `enable opening edge (t = ${latchOpenT.toPrecision(3)} ` +
                `${timeUnit}) — latch became transparent here`);
        }

        // ── Per-lane bar bands ──────────────────────────────────────────
        const BAR_OPA = DIAGRAM_CONST.BAR_OPACITY;
        // Plain-English explainer per band kind, used below to attach a
        // hover tooltip. Each call site picks the right kind for the
        // band's role on this lane (input/output) — same color reused
        // with different copy when needed.
        const bandTip = (kind, role, t1, t2) => {
            const span = `${t1.toPrecision(3)} → ${t2.toPrecision(3)}${
                ' '}${timeUnit} (${(t2 - t1).toPrecision(3)} ${timeUnit} wide)`;
            const desc = (
                kind === 'lib-setup' ? `library setup time (tsu) — D must be ` +
                    `stable at least this long before the capture edge`
                : kind === 'lib-hold' ? `library hold time (th) — D must ` +
                    `stay stable at least this long after the capture edge`
                : kind === 'setup-unc' ? `setup clock uncertainty (CTS / ` +
                    `jitter margin) reserved before the capture edge`
                : kind === 'hold-unc' ? `hold clock uncertainty (CTS / ` +
                    `jitter margin) reserved after the capture edge`
                : kind === 'uncert' && role === 'clk-to-q' ? `clock-to-Q ` +
                    `delay region — Q is not yet definitely valid`
                : kind === 'uncert' && role === 'borrow' ? `time-borrow ` +
                    `window — late D arrival is acceptable here`
                : kind === 'valid' ? `Q is stable at a known value`
                : kind === 'invalid' ? `D is free to change — no setup/hold ` +
                    `restriction in this region`
                : kind);
            return `${kind}: ${span}\n${desc}`;
        };
        const drawBar = (laneTop, t1, t2, kind, role) => {
            const x1 = Math.max(LABEL_W, tx(t1));
            const x2 = Math.min(totalW, tx(t2));
            const rect = this._svgBandRect(svg, x1, laneTop, x2 - x1, LANE_H,
                                           kind, BAR_OPA);
            if (rect && x2 - x1 > 1) this._svgTitle(rect, bandTip(kind, role, t1, t2));
            return rect;
        };

        for (let i = 0; i < lanes.length; i++) {
            const L = lanes[i];
            const top = LANE_TOP_BASE + i * (LANE_H + LANE_GAP);

            // Left label column: bus/pin name + compact constraint line.
            // Click the name to inspect (only for single-pin lanes —
            // bus-collapsed lanes don't have a single ODB target).
            const nameY = top + LANE_H / 2 - 1;
            // Non-endpoint instance pins are rendered with a dimmer label
            // so the user can tell at a glance which pins anchor a real
            // setup/hold check vs. which are just shown for context.
            const labelColor = L.isEndpoint
                ? 'var(--fg-primary)' : 'var(--fg-muted)';
            const nameEl = this._svgText(svg, 4, nameY,
                this._truncateForLane(L.label, LABEL_W - 8),
                labelColor, 9, 'start');
            // Tooltip: full label + folded-in pins for bus lanes.
            this._svgTitle(nameEl, L.titleText || L.label);
            if (L.linkPin && L.count === 1) {
                // Single-pin lane: click → inspect that pin. Endpoint-detail
                // emits ODB refs as `name_odb_type`/`name_odb_id` so we use
                // the prefixed form of _linkifyPin.
                this._linkifyPin(nameEl, L.linkPin, 'name');
            } else if (L.busRoot && L.count > 1 && onBusExpand) {
                // Bus-collapsed lane: click → expand into individual lanes.
                // Each individual pin becomes selectable in the re-render.
                nameEl.style.cursor = 'pointer';
                nameEl.style.textDecoration = 'underline';
                nameEl.style.textDecorationStyle = 'dotted';
                nameEl.style.textDecorationColor = 'var(--fg-muted)';
                nameEl.addEventListener('click', (e) => {
                    e.stopPropagation();
                    onBusExpand(L.busRoot);
                });
            }

            if (L.kind === 'output') {
                // ── Output lane: clk-to-Q delay region ────────────────
                // Capture edge at t=0 is the launching edge. Until qMax
                // the output is in its clk→Q delay region — Q is not
                // yet definitely valid. Drawn as a single band from t=0
                // to qMax in the uncert (yellow) palette so the entire
                // delay shows even when qMin is small. If qMin > 0 we
                // overlay a dotted marker at qMin to call out "earliest
                // possible transition". After qMax the output is valid
                // (green) at the new value.
                const qMin = Math.max(0, L.clkqMin || 0);
                const qMax = Math.max(qMin, L.clkqMax || 0);
                if (LANE_H >= 18 && (qMin > 0 || qMax > 0)) {
                    const parts = [];
                    if (qMin > 0) parts.push(`clk→Q≥${qMin.toPrecision(2)}`);
                    if (qMax > 0) parts.push(`clk→Q≤${qMax.toPrecision(2)}`);
                    if (parts.length > 0) {
                        this._svgText(svg, 4, top + LANE_H / 2 + 9,
                            this._truncateForLane(parts.join(' '), LABEL_W - 8),
                            'var(--fg-muted)', 7, 'start');
                    }
                }
                // Previous-cycle stable Q value before this launch edge.
                drawBar(top, tStart, 0, 'valid');
                // clk→Q delay region: t=0 → qMax, Q not definitely valid.
                if (qMax > 0) drawBar(top, 0, qMax, 'uncert', 'clk-to-q');
                // qMin marker: dotted vertical line at the earliest
                // possible transition time. Helps the user see that the
                // delay band has both a lower (qMin) and upper (qMax) edge.
                if (qMin > 0 && qMin < qMax) {
                    const xq = tx(qMin);
                    const m = this._svgLine(svg, xq, top + 1, xq, top + LANE_H - 1,
                                            'var(--canvas-axis)', '0.8');
                    m.setAttribute('stroke-dasharray', '2,2');
                }
                drawBar(top, qMax, tEnd, 'valid');
                this._svgOutlineRect(svg, LABEL_W, top, WAVE_W, LANE_H);
                continue;
            }

            // ── Input lane: setup/hold around capture edge at t=0 ─────
            const tsu = L.tsu, th = L.th, su = L.su, hu = L.hu;
            const thAbs = Math.abs(th);
            const tSetupDeadline = -(tsu + su);
            const tHoldVisual    =  hu + thAbs;
            // Time-borrow on latch input lanes: late D arrival up to
            // `borrow` past the close edge is acceptable (the next
            // stage absorbs the slack). When the borrow window extends
            // past the hold band, draw [tHoldVisual, borrow] in the
            // uncert (yellow) palette so the user can see the borrowed
            // region distinct from the unconstrained "free" region
            // beyond it.
            const borrow = laneEffectiveBorrow(L);
            const tBorrowEnd = Math.max(tHoldVisual, borrow);
            // Compact tsu/th annotation under the name (only when there's
            // room — otherwise the lane is too dense and the user can
            // still read the band widths).
            if (LANE_H >= 18) {
                const parts = [];
                if (tsu > 0)   parts.push(`${TIMING_LABELS.tsu.short}=${tsu.toPrecision(2)}`);
                if (su > 0)    parts.push(`${TIMING_LABELS.setup_unc.short}=${su.toPrecision(2)}`);
                if (hu > 0)    parts.push(`${TIMING_LABELS.hold_unc.short}=${hu.toPrecision(2)}`);
                if (thAbs > 0) parts.push(`${TIMING_LABELS.th.short}=${thAbs.toPrecision(2)}`);
                if (borrow > tHoldVisual) {
                    const tag = (L.timeBorrowLimit != null) ? 'borrow' : 'borrow*';
                    parts.push(`${tag}=${borrow.toPrecision(2)}`);
                }
                if (parts.length > 0) {
                    this._svgText(svg, 4, top + LANE_H / 2 + 9,
                        this._truncateForLane(parts.join(' '), LABEL_W - 8),
                        'var(--fg-muted)', 7, 'start');
                }
            }
            // Bar bands. Use the gray "invalid"/free palette outside the
            // setup/hold window so the input "free to change" region is
            // visually distinct from the green "valid/stable" region used
            // on output lanes — the two readings are different (input D
            // doesn't have to be at any particular value here, vs. output
            // Q is definitely settled).
            drawBar(top, tStart, tSetupDeadline, 'invalid');
            if (tsu > 0)   drawBar(top, tSetupDeadline, -su, 'lib-setup');
            if (su > 0)    drawBar(top, -su, 0, 'setup-unc');
            if (hu > 0)    drawBar(top, 0, hu, 'hold-unc');
            if (thAbs > 0) drawBar(top, hu, tHoldVisual, 'lib-hold');
            if (borrow > tHoldVisual) {
                drawBar(top, tHoldVisual, tBorrowEnd, 'uncert', 'borrow');
            }
            drawBar(top, tBorrowEnd, tEnd, 'invalid');
            // Outline.
            this._svgOutlineRect(svg, LABEL_W, top, WAVE_W, LANE_H);
        }

        // ── Latch transparent-period overlay ────────────────────────────
        // While the enable is asserted (open→close) the latch is
        // transparent: D flows directly to Q. Draw a hatched overlay
        // spanning that window across every data lane so the user can
        // see at a glance where the latch is "live". Setup/hold bands
        // remain visible underneath; the overlay is annotation only.
        if (isLatch && latchOpenT < 0) {
            const xOpen = Math.max(LABEL_W, tx(latchOpenT));
            const xClose = Math.min(totalW, tx(0));
            if (xClose > xOpen) {
                const lanesTop = LANE_TOP_BASE;
                const lanesH   = lanes.length * (LANE_H + LANE_GAP) - LANE_GAP;
                const patternId = `sdc-ep-latch-stripes-${
                    Math.floor(Math.random() * 1e9).toString(36)}`;
                let defs = svg.querySelector('defs');
                if (!defs) {
                    defs = document.createElementNS(SVG_NS, 'defs');
                    svg.insertBefore(defs, svg.firstChild);
                }
                const pat = document.createElementNS(SVG_NS, 'pattern');
                pat.setAttribute('id', patternId);
                pat.setAttribute('patternUnits', 'userSpaceOnUse');
                pat.setAttribute('width', '6');
                pat.setAttribute('height', '6');
                pat.setAttribute('patternTransform', 'rotate(45)');
                const stripe = document.createElementNS(SVG_NS, 'line');
                stripe.setAttribute('x1', '0'); stripe.setAttribute('y1', '0');
                stripe.setAttribute('x2', '0'); stripe.setAttribute('y2', '6');
                stripe.setAttribute('stroke', 'var(--canvas-axis)');
                stripe.setAttribute('stroke-width', '1.5');
                stripe.setAttribute('stroke-opacity', '0.35');
                pat.appendChild(stripe);
                defs.appendChild(pat);

                const overlay = document.createElementNS(SVG_NS, 'rect');
                overlay.setAttribute('x', xOpen);
                overlay.setAttribute('y', lanesTop);
                overlay.setAttribute('width', xClose - xOpen);
                overlay.setAttribute('height', lanesH);
                overlay.setAttribute('fill', `url(#${patternId})`);
                overlay.setAttribute('pointer-events', 'none');
                overlay.classList.add('sdc-ep-latch-overlay');
                this._svgTitle(overlay,
                    'latch transparent period — D flows to Q while the ' +
                    'enable is asserted; data is captured at the close edge');
                svg.appendChild(overlay);

                // Compact "transparent" label at the top of the overlay
                // (right after the open edge), placed only if there's
                // enough horizontal room.
                if (xClose - xOpen > 60) {
                    this._svgText(svg, xOpen + 4, lanesTop - 2,
                        'transparent', 'var(--fg-muted)', 8, 'start');
                }
            }
        }

        // ── Shared axis: minor ticks + labelled ticks ──────────────────
        this._svgLine(svg, LABEL_W, AXIS_Y, totalW, AXIS_Y, 'var(--canvas-axis)', 1);

        // Minor (unlabelled) ticks at "nice" intervals — gives the user
        // a visual ruler for reading off durations between bands without
        // crowding the axis with numbers. Step is picked from
        // {1,2,5}×10^k to land roughly 8–12 ticks across the displayed
        // range; render as short faint marks below the axis line.
        const niceStep = (range) => {
            const target = range / 10;
            const exp = Math.floor(Math.log10(target));
            const base = target / Math.pow(10, exp);
            const mult = base < 1.5 ? 1 : base < 3.5 ? 2 : base < 7.5 ? 5 : 10;
            return mult * Math.pow(10, exp);
        };
        const minorStep = niceStep(tRange);
        if (minorStep > 0 && Number.isFinite(minorStep)) {
            const tFirst = Math.ceil(tStart / minorStep) * minorStep;
            for (let t = tFirst; t <= tEnd + 1e-9; t += minorStep) {
                if (Math.abs(t) < minorStep * 1e-6) continue;  // skip ~0 (covered by labelled tick)
                const x = tx(t);
                if (x < LABEL_W || x > totalW) continue;
                this._svgLine(svg, x, AXIS_Y, x, AXIS_Y + 2,
                              'var(--canvas-axis)', 0.5);
            }
        }

        // Labelled ticks: collect first, then drop any that would
        // visually overlap a higher-priority neighbour. Priority order:
        //   1) capture edge ("0")               — always shown
        //   2) period bookend (T=…)             — always shown
        //   3) outer setup/hold boundaries      — dropped first when crowded
        // A tick is "overlapping" if its label box (x ± half-width + 2px
        // pad) intersects a previously-placed neighbour's box.
        const addTick = (t, label, row, priority, tip) => ({
            x: Math.max(LABEL_W + 3, Math.min(totalW - 3, tx(t))),
            label, row, priority, tip,
        });
        const candidates = [
            addTick(0, '0', 1, 0, isLatch
                ? 'capture/closing edge — D is captured here'
                : 'capture edge — t = 0'),
            addTick(tStart, `T=${T.toPrecision(4)}${timeUnit}`, 1, 1,
                `previous launch edge — one period (T = ${T.toPrecision(4)}${
                    timeUnit}) before the capture edge`),
        ];
        if (maxTsuSu > 0)
            candidates.push(addTick(-maxTsuSu, `${(-maxTsuSu).toPrecision(3)}`,
                0, 2, `outer setup boundary — D must be stable starting here ` +
                `(tsu + setup uncertainty = ${maxTsuSu.toPrecision(3)} ${timeUnit})`));
        if (maxThHu > 0)
            candidates.push(addTick(maxThHu, `${maxThHu.toPrecision(3)}`,
                0, 2, `outer hold boundary — D must remain stable until here ` +
                `(th + hold uncertainty = ${maxThHu.toPrecision(3)} ${timeUnit})`));
        if (maxClkqMax > 0 && maxClkqMax > maxThHu)
            candidates.push(addTick(maxClkqMax, `${maxClkqMax.toPrecision(3)}`,
                0, 2, `clk-to-Q upper bound — Q is definitely valid past here`));
        if (maxBorrow > 0 && maxBorrow > maxThHu && maxBorrow !== maxClkqMax)
            candidates.push(addTick(maxBorrow, `${maxBorrow.toPrecision(3)}`,
                0, 2, `time-borrow boundary — late D arrival up to here is ` +
                `acceptable (set_max_time_borrow or transparent-period default)`));
        candidates.sort((a, b) => a.priority - b.priority);
        const placed = [];
        for (const c of candidates) {
            const half = c.label.length * 2.5;
            const left = c.x - half - 2, right = c.x + half + 2;
            const collides = placed.some(p =>
                p.row === c.row && !(right < p.left || left > p.right));
            if (collides) continue;
            placed.push({ ...c, left, right });
        }
        for (const { x, label, row, tip } of placed) {
            const tickLine = this._svgLine(svg, x, AXIS_Y - 2, x, AXIS_Y + 3,
                'var(--canvas-axis)', 1);
            if (tip) this._svgTitle(tickLine, tip);
            const ty   = TICK_ROW[row];
            const half = label.length * 2.5;
            const lx2  = Math.max(LABEL_W + half + 1, Math.min(totalW - half - 1, x));
            const txt = this._svgText(svg, lx2, ty, label,
                'var(--canvas-label)', 8, 'middle');
            if (tip) this._svgTitle(txt, tip);
        }

        // ── Legend ──────────────────────────────────────────────────────
        // Driven by what's actually painted: only show categories any lane
        // contributes (tsu / Su / Hu / |th| can be empty).
        const COL_VALID     = BAND_COLORS['valid'];
        const COL_INVALID   = BAND_COLORS['invalid'];
        const COL_SETUP_UNC = BAND_COLORS['setup-unc'];
        const COL_HOLD_UNC  = BAND_COLORS['hold-unc'];
        const COL_LIB_SETUP = BAND_COLORS['lib-setup'];
        const COL_LIB_HOLD  = BAND_COLORS['lib-hold'];
        const COL_UNCERT    = BAND_COLORS['uncert'];
        const anyTsu  = inputLanes.some(L => L.tsu > 0);
        const anySu   = inputLanes.some(L => L.su  > 0);
        const anyHu   = inputLanes.some(L => L.hu  > 0);
        const anyTh   = inputLanes.some(L => Math.abs(L.th) > 0);
        const anyClkq = outputLanes.some(L => (L.clkqMax || 0) > 0
                                              || (L.clkqMin || 0) > 0);
        const anyInput  = inputLanes.length > 0;
        const anyOutput = outputLanes.length > 0;
        const anyBorrow = isLatch && inputLanes.some(L => {
            const b = laneEffectiveBorrow(L);
            return b > (Math.abs(L.th) + L.hu);
        });
        const anyExplicitBorrow = isLatch && inputLanes.some(
            L => L.timeBorrowLimit != null && L.timeBorrowLimit > 0);
        // Two semantic flavours for the "outer" bands on a lane:
        //   • input  D pin: gray "free" — data may change, no constraint
        //   • output Q pin: green "valid" — Q is stable at a known value
        // We only surface a legend item when the corresponding lane kind
        // is actually present, so single-direction diagrams stay simple.
        const freeTip  = 'input data may change — no setup/hold restriction in this region';
        const validTip = 'output Q is stable at a known value here';
        const legItems = [
            ...(anyInput  ? [{ color: COL_INVALID, label: 'free',  tip: freeTip  }] : []),
            ...(anyOutput ? [{ color: COL_VALID,   label: 'valid', tip: validTip }] : []),
            ...(anyTsu ? [{ color: COL_LIB_SETUP,
                            label: TIMING_LABELS.tsu.short,
                            tip: TIMING_LABELS.tsu.tip }] : []),
            ...(anySu  ? [{ color: COL_SETUP_UNC,
                            label: TIMING_LABELS.setup_unc.short,
                            tip: TIMING_LABELS.setup_unc.tip }] : []),
            ...(anyHu  ? [{ color: COL_HOLD_UNC,
                            label: TIMING_LABELS.hold_unc.short,
                            tip: TIMING_LABELS.hold_unc.tip }] : []),
            ...(anyTh  ? [{ color: COL_LIB_HOLD,
                            label: TIMING_LABELS.th.short,
                            tip: TIMING_LABELS.th.tip }] : []),
            ...(anyClkq ? [{ color: COL_UNCERT,
                             label: 'clk→Q',
                             tip: 'clock-to-Q delay region — Q may transition anywhere from the launch edge until clk→Q max; output is valid only after this band' }] : []),
            ...(anyBorrow ? [{ color: COL_UNCERT,
                               label: anyExplicitBorrow ? 'borrow' : 'borrow*',
                               tip: anyExplicitBorrow
                                 ? 'time borrow — late D arrival up to set_max_time_borrow past the close edge is acceptable; the next stage absorbs the slack'
                                 : 'time borrow (implicit, transparent-period default) — late D arrival is acceptable up to the latch open edge' }] : []),
            ...(isLatch ? [{ pattern: 'stripes',
                             label: 'transparent',
                             tip: 'latch transparent period — enable asserted, D flows to Q; capture happens at the close edge' }] : []),
        ];
        const legW = (label) => label.length * 5.5 + 16;
        let lx = LABEL_W + 4;
        for (const item of legItems) {
            const { color, pattern, label, tip } = item;
            const tw = legW(label);
            if (lx + tw > totalW - 4) break;
            const swatch = document.createElementNS(SVG_NS, 'rect');
            swatch.setAttribute('x', lx); swatch.setAttribute('y', LEG_Y - 7);
            swatch.setAttribute('width', 8); swatch.setAttribute('height', 8);
            if (pattern === 'stripes') {
                // Reuse the same hatched pattern as the lane overlay so
                // the legend swatch matches what the user sees on lanes.
                const ovr = svg.querySelector('.sdc-ep-latch-overlay');
                const fillRef = ovr ? ovr.getAttribute('fill') : null;
                if (fillRef) swatch.setAttribute('fill', fillRef);
                else         swatch.style.fill = 'var(--canvas-axis)';
                // Outline so the swatch reads even with a sparse pattern.
                swatch.setAttribute('stroke', 'var(--canvas-axis)');
                swatch.setAttribute('stroke-width', '0.5');
            } else {
                swatch.style.fill = color;
                swatch.style.fillOpacity = BAR_OPA;
            }
            this._svgTitle(swatch, tip);
            svg.appendChild(swatch);
            const t = this._svgText(svg, lx + 10, LEG_Y, label,
                                    'var(--canvas-label)', 8, 'start');
            this._svgTitle(t, tip);
            lx += tw;
        }

        return svg;
    }

    // Group pins for the multi-lane endpoint diagram. Pins that share
    // a constraint signature (same tsu/th/su/hu on this clock) AND a
    // bus-style suffix (`name[N]`) collapse into a single lane; everything
    // else stays as its own lane.
    //
    // `expandedBuses` (optional) is a Set of bus root names that the user
    // has explicitly expanded — those buses skip collapsing and emit one
    // lane per pin instead. Used to implement the inline-expand behavior
    // on bus-collapsed lanes (click ▶ next to the bus label to drill in).
    //
    // The returned lane carries:
    //   { label, titleText, tsu, th, su, hu, pins, count, linkPin,
    //     busRoot? }   busRoot present iff the lane is a bus group AND
    //                  count > 1 (so the diagram knows it's collapsible).
    _buildEndpointLanes(pins, clkInfo, expandedBuses, instancePath) {
        const expanded = expandedBuses instanceof Set ? expandedBuses : new Set();
        const clkName = clkInfo && clkInfo.name;
        // Lane direction: 'output' if the pin's library port is an output
        // (drives clk-to-Q), else 'input' (setup/hold). Bidirect/internal
        // fall back to input semantics — same lane shape, just less common.
        const dirOf = (p) =>
            (p.direction === 'output' || p.direction === 'tristate')
                ? 'output' : 'input';
        const clkqRange = (p) => {
            const q = p.clk_to_q;
            if (!q) return { min: 0, max: 0, has: false };
            const candMax = [q.rise_max, q.fall_max].filter(v => v != null);
            const candMin = [q.rise_min, q.fall_min].filter(v => v != null);
            const max = candMax.length ? Math.max(...candMax) : 0;
            const min = candMin.length ? Math.min(...candMin) : 0;
            return { min, max, has: candMax.length > 0 || candMin.length > 0 };
        };
        const sigOf = (p) => {
            const dir = dirOf(p);
            if (dir === 'output') {
                const q = clkqRange(p);
                // Output lanes share a lane only when their clk-to-Q
                // window matches; otherwise visualisation would lie.
                return `out|${q.min.toPrecision(6)}|${q.max.toPrecision(6)}`;
            }
            const c = (p.clocks || []).find(x => x.name === clkName) || {};
            // Time-borrow limit is part of the lane signature so a
            // latch D pin with set_max_time_borrow doesn't share a
            // lane with one without — the borrow band would lie if it
            // collapsed pins with different limits.
            return [
                'in',
                c.library_setup     || 0,
                c.library_hold      || 0,
                c.uncertainty_setup || 0,
                c.uncertainty_hold  || 0,
                c.time_borrow_limit != null ? c.time_borrow_limit : 'def',
            ].join('|');
        };
        const busOf = (name) => {
            const m = name && name.match(/^(.*)\[(\d+)\]$/);
            return m ? { root: m[1], index: +m[2] } : null;
        };
        const groups = new Map();
        const groupOrder = [];  // preserve first-seen order
        for (const p of pins) {
            const sig = sigOf(p);
            const bus = busOf(p.name);
            // Force-expanded buses skip group key so each pin lives on
            // its own lane.
            const useBusKey = bus && !expanded.has(bus.root);
            const key = useBusKey ? `bus|${sig}|${bus.root}` : `solo|${p.name}`;
            if (!groups.has(key)) {
                groups.set(key, { sig, bus: useBusKey ? bus : null,
                                  pins: [], indices: [] });
                groupOrder.push(key);
            }
            const g = groups.get(key);
            g.pins.push(p);
            if (useBusKey) g.indices.push(bus.index);
        }
        const lanes = [];
        for (const key of groupOrder) {
            const g = groups.get(key);
            const first = g.pins[0];
            const dir = dirOf(first);
            const c = (first.clocks || []).find(x => x.name === clkName) || {};
            const tsu = c.library_setup     || 0;
            const th  = c.library_hold      || 0;
            const su  = c.uncertainty_setup || 0;
            const hu  = c.uncertainty_hold  || 0;
            const q   = clkqRange(first);
            // null = no explicit set_max_time_borrow (frontend may still
            // draw the implicit transparent-period borrow on latches).
            const tbl = (c.time_borrow_limit != null
                         && Number.isFinite(c.time_borrow_limit))
                ? c.time_borrow_limit : null;
            // Endpoint flag is true if every pin folded into the lane is a
            // graph endpoint; bus lanes drop the flag if any pin in the
            // bundle is non-endpoint, so the lane is rendered with the
            // dimmer "non-endpoint" tone.
            const isEndpoint = g.pins.every(p => p._isEndpoint !== false
                                                 && p._isEndpoint !== undefined);
            let label, titleText, linkPin = null, busRoot = null;
            if (g.bus && g.pins.length > 1) {
                const sorted = [...g.indices].sort((a, b) => a - b);
                const min = sorted[0], max = sorted[sorted.length - 1];
                const contiguous = sorted.length === (max - min + 1);
                // Surrounding instance card already names the
                // instance, so trim the bus root to its leaf — full
                // path lives on the lane's `titleText` for hover.
                const rootLeaf = this._pinLeafName(g.bus.root, instancePath);
                label = contiguous
                    ? `▶ ${rootLeaf}[${max}:${min}]  (${g.pins.length} pins)`
                    : `▶ ${rootLeaf}  (${g.pins.length} pins)`;
                const sortedPins = [...g.pins].sort((a, b) => {
                    const ai = busOf(a.name).index, bi = busOf(b.name).index;
                    return ai - bi;
                });
                titleText = `Bus group — click to expand into individual lanes\n\n` +
                    sortedPins.map(p => p.name).join('\n');
                busRoot = g.bus.root;
            } else {
                label = this._pinLeafName(first.name, instancePath);
                titleText = first.name;
                linkPin = first;
            }
            lanes.push({
                kind: dir,                // 'input' | 'output'
                label, titleText,
                tsu, th, su, hu,          // input-lane shape
                timeBorrowLimit: tbl,     // explicit set_max_time_borrow, ns
                clkqMin: q.min, clkqMax: q.max, hasClkq: q.has,  // output-lane shape
                isEndpoint,
                pins: g.pins, count: g.pins.length, linkPin, busRoot,
            });
        }
        return lanes;
    }

    // Pick the most appropriate scroll-area width for a Port-Delays
    // diagram. Prefer the dedicated PD pane when it's mounted; fall
    // back to the Endpoint result pane (which embeds the same diagram
    // inside a deeply-nested card → larger pad deduction); fall back
    // to 600 px when nothing is laid out yet (initial render before
    // the panel is in the DOM).
    _pdDiagramContainerW() {
        if (this._pdScrollArea && this._pdScrollArea.clientWidth > 0)
            return this._pdScrollArea.clientWidth - 16;
        if (this._epResultArea && this._epResultArea.clientWidth > 0)
            return this._epResultArea.clientWidth - 52;
        return 600;
    }

    // Build the displayed clock-edge list for a multi-period diagram
    // (single-pin and multi-lane endpoint variants both need it). Edges
    // are shifted so that `tRef` lands at t=0 in the diagram's own
    // axis, then sampled across [-2T..+2T] periods and clipped to the
    // visible window — that's enough to cover any reasonable
    // [tStart,tEnd] range without computing the bounds analytically.
    //
    // Returns { edges, prevY }:
    //   edges  — sorted [{ t, isRise }] entries inside [tStart, tEnd]
    //   prevY  — the clock level just before tStart (CLK_HIGH/CLK_LOW),
    //            so callers can seed _drawClockPath without re-walking
    //            the waveform.
    _collectClockEdges(waveform, T, tRef, tStart, tEnd) {
        const { CLK_HIGH, CLK_LOW } = DIAGRAM_CONST;
        const edges = [];
        for (let period = -2; period <= 2; period++) {
            for (let i = 0; i < waveform.length; i++) {
                const et = waveform[i] - tRef + period * T;
                if (et > tStart - 1e-9 && et < tEnd + 1e-9) {
                    edges.push({ t: et, isRise: (i % 2 === 0) });
                }
            }
        }
        edges.sort((a, b) => a.t - b.t);
        // Walk earlier periods back-to-front so prevY ends on the most
        // recent transition that occurred before tStart.
        let prevY = CLK_HIGH;
        for (let period = -10; period <= 0; period++) {
            for (let i = 0; i < waveform.length; i++) {
                const et = waveform[i] - tRef + period * T;
                if (et < tStart) prevY = (i % 2 === 0) ? CLK_HIGH : CLK_LOW;
            }
        }
        return { edges, prevY };
    }

    // Trim a label to roughly fit the lane label column. Approximates
    // 5.5 px per char (SVG monospace 9pt) — close enough for clipping
    // decisions; the full text is always available via the <title>.
    _truncateForLane(text, maxPx) {
        if (!text) return '';
        const maxChars = Math.floor(maxPx / 5.5);
        return text.length <= maxChars
            ? text
            : text.slice(0, Math.max(3, maxChars - 1)) + '…';
    }

    // Build a labeled section card for the endpoint result view.
    _makeEpSection(title, fillBody) {
        const sec = document.createElement('div');
        sec.style.cssText =
            'border:1px solid var(--border);border-radius:4px;overflow:hidden;';

        const hdr = document.createElement('div');
        hdr.style.cssText =
            'padding:4px 10px;background:var(--bg-header);font-size:12px;font-weight:600;' +
            'color:var(--fg-primary);border-bottom:1px solid var(--border);';
        hdr.textContent = title;
        sec.appendChild(hdr);

        const body = document.createElement('div');
        fillBody(body);
        sec.appendChild(body);

        return sec;
    }

    // ── CDC tab ───────────────────────────────────────────────────────────────
    //
    // The CDC tab does timing-graph traversal, not constraint browsing —
    // it lives in this widget because the SDC clock-groups + exceptions
    // it cross-references are already loaded by the other tabs. Three
    // levels:
    //   Level 1   matrix of (launch × capture) clock pairs, each cell
    //             showing path counts colour-coded by category
    //   Level 2   per-cell paginated list of CDC paths, with a "Sync
    //             chain" column explaining how each path was classified
    //   Level 3   per-path stage diagram showing the capture flop and
    //             any sync chain following it
    //
    // The Level-1 scan walks every endpoint in the analyser, so it's
    // gated behind a "▶ Scan CDC" button (same UX as the Endpoints tab).

    _buildCdcPanel(container) {
        container.style.cssText =
            'display:flex;flex-direction:column;height:100%;overflow:hidden;';

        // ── Toolbar ──────────────────────────────────────────────────────
        const toolbar = document.createElement('div');
        toolbar.style.cssText =
            'display:flex;align-items:center;padding:4px 8px;gap:6px;' +
            'border-bottom:1px solid var(--border);background:var(--bg-header);' +
            'flex-shrink:0;flex-wrap:wrap;';

        const scanBtn = document.createElement('button');
        scanBtn.textContent = '▶ Scan CDC';
        scanBtn.title =
            'Walk every register endpoint, find launch & capture clocks, and ' +
            'classify each clock crossing as synchronized / excluded / ' +
            'unsynchronized. Cached after the first call. On a million-' +
            'instance design this can take several seconds — same cost as ' +
            'the Endpoints "List endpoints" button.';
        scanBtn.style.cssText =
            'padding:3px 10px;font-size:12px;cursor:pointer;background:var(--accent-tab);' +
            'color:#fff;border:none;border-radius:3px;white-space:nowrap;';
        scanBtn.addEventListener('click', () => this._loadCdcOverview());
        this._cdcScanBtn = scanBtn;
        toolbar.appendChild(scanBtn);

        const refreshBtn = document.createElement('button');
        refreshBtn.textContent = '↺ Refresh';
        refreshBtn.title =
            'Re-walk the endpoint set and re-classify every CDC path. Use ' +
            'this after loading new SDC, re-linking the design, or changing ' +
            'the synchronizer whitelist.';
        refreshBtn.style.cssText =
            'display:none;padding:3px 10px;font-size:12px;cursor:pointer;' +
            'background:var(--bg-input);color:var(--fg-primary);' +
            'border:1px solid var(--border);border-radius:3px;white-space:nowrap;';
        refreshBtn.addEventListener('click', () => this._loadCdcOverview());
        this._cdcRefreshBtn = refreshBtn;
        toolbar.appendChild(refreshBtn);

        // CDC settings — opens the whitelist panel.
        const settingsBtn = document.createElement('button');
        settingsBtn.textContent = '⚙ CDC settings';
        settingsBtn.title =
            'Edit the synchronizer whitelist. Two glob-pattern lists: ' +
            'instance paths (matched against the capture flop\'s instance ' +
            'name) and master cell names (matched against the capture flop\'s ' +
            'liberty cell). Either match force-classifies the path as ' +
            '"synchronized (whitelisted)". Session-scoped — not persisted.';
        settingsBtn.style.cssText =
            'padding:3px 10px;font-size:12px;cursor:pointer;background:var(--bg-input);' +
            'color:var(--fg-primary);border:1px solid var(--border);' +
            'border-radius:3px;white-space:nowrap;';
        settingsBtn.addEventListener('click', () => this._openCdcSettings());
        toolbar.appendChild(settingsBtn);

        // Status pill — endpoint count after a successful scan.
        const status = document.createElement('span');
        status.style.cssText =
            'margin-left:auto;font-size:12px;color:var(--fg-muted);' +
            'font-style:italic;';
        this._cdcStatus = status;
        toolbar.appendChild(status);

        container.appendChild(toolbar);

        // ── Body: three stacked sub-panels (only one visible at a time) ──
        // `overflow:auto` (both axes) so the matrix view's sticky
        // headers have ONE scroll container to anchor to — using
        // `overflow-y:auto` alone left horizontal sticky orphaned
        // when `wrap` was the (implicit) horizontal scroll container
        // but had no height constraint, so vertical sticky never
        // activated. Single scroll container fixes both axes
        // simultaneously.
        const body = document.createElement('div');
        body.style.cssText =
            'flex:1;overflow:auto;padding:8px;background:var(--bg-main);';
        this._cdcBody = body;
        container.appendChild(body);

        // Initial empty-state message.
        body.innerHTML =
            '<div style="padding:24px;color:var(--fg-muted);font-style:italic;">' +
            'Click "▶ Scan CDC" to walk every register endpoint and ' +
            'classify each clock-domain crossing.</div>';

        // Sticky-bottom legend footer — populated only when the
        // path-detail view is visible (the colour-tinted stage cards
        // and various banner colours need a key). Sibling of the
        // scroll body, with `flex-shrink:0` so the panel's flex
        // column lays it as a footer below the scroll area. Hidden
        // by default; `_renderCdcPathDetail` toggles it on, the
        // matrix and path-list views toggle it back off.
        const legend = document.createElement('div');
        legend.style.cssText
            = 'flex-shrink:0;border-top:1px solid var(--border);'
            + 'background:var(--bg-header);'
            + 'padding:6px 10px;'
            + 'font-size:11px;color:var(--fg-muted);'
            + 'display:none;'
            + 'flex-direction:column;gap:4px;'
            + 'max-height:120px;overflow-y:auto;';
        this._cdcLegendFooter = legend;
        container.appendChild(legend);

        // Sub-pane handles, populated by load methods.
        this._cdcMatrixPane = null;
        this._cdcPathsPane  = null;
        this._cdcDetailPane = null;

        // Currently-selected drill-down state. Used by Refresh and by the
        // settings-apply path so the user lands back on the same view.
        this._cdcCurrentLaunch  = null;
        this._cdcCurrentCapture = null;
        this._cdcCurrentCategoryFilter = 'all';
    }

    async _loadCdcOverview() {
        if (this._cdcLoading) return;
        this._cdcLoading = true;
        this._cdcBody.innerHTML =
            '<div style="padding:24px;color:var(--fg-muted);font-style:italic;">' +
            'Scanning endpoints across every mode…</div>';
        try {
            await this._app.websocketManager.readyPromise;
            // The overview response carries data for *every* mode at
            // once, so a subsequent mode switch is a re-render from
            // cache — no second fetch needed.
            const data = await this._requestWithTimeout({ type: 'cdc_overview' });
            this._cdcOverview = data;
            // Mix-endpoint listing is keyed on the same scan. Drop the
            // cache so a Refresh re-walks the design instead of
            // re-rendering stale origins.
            this._cdcMixEndpointsCache = null;
            this._renderCdcMatrix(data);
            // After the first successful scan, swap "Scan" for "Refresh".
            if (this._cdcScanBtn) this._cdcScanBtn.style.display = 'none';
            if (this._cdcRefreshBtn) this._cdcRefreshBtn.style.display = '';
        } catch (e) {
            console.warn('[CDC] overview load failed', e);
            this._cdcBody.innerHTML =
                `<div style="padding:24px;color:var(--err-fg);">` +
                `Failed to load CDC overview: ${(e && e.message) || e}</div>`;
        } finally {
            this._cdcLoading = false;
        }
    }

    // Pick which mode's matrix to display.
    //
    // Priority order matters: `_currentModeName` is updated on every
    // `sdc_set_mode` round-trip (see _setMode), so it's the freshest
    // signal and must be checked first. `data.current_mode` is a
    // snapshot from when the overview was fetched — stale after a
    // mode switch, which is exactly when this function gets called.
    _cdcActiveModeName(data) {
        if (!data || !data.modes) return '';
        const modes = data.modes;
        if (this._currentModeName && modes[this._currentModeName]) {
            return this._currentModeName;
        }
        if (data.current_mode && modes[data.current_mode]) {
            return data.current_mode;
        }
        const keys = Object.keys(modes);
        return keys.length ? keys[0] : '';
    }

    // Render the (launch × capture) matrix. Cell colour reflects the most
    // severe category present:
    //   any unsynced path     → red
    //   any sync but no unsync → green
    //   only excluded         → grey
    //   empty                 → dim border
    _renderCdcMatrix(data) {
        this._cdcBody.innerHTML = '';
        // Hide the path-detail legend footer — it carries
        // path-specific clock+banner annotations that don't apply
        // to the matrix view.
        this._hideCdcLegendFooter();
        // Drop any stale path-list state — the tbody DOM node we
        // were appending to is gone now that we're back on the
        // matrix. A pending _fetchCdcPathsMore callback uses these
        // refs and the token to no-op safely after navigation.
        this._cdcPathsTbody  = null;
        this._cdcPathsFooter = null;
        this._cdcPathsToken  = (this._cdcPathsToken || 0) + 1;
        const wrap = document.createElement('div');
        // `wrap` deliberately has NO overflow — `_cdcBody` is the
        // single scroll container for the matrix view (both axes).
        // Giving wrap its own `overflow-x:auto` would make IT the
        // horizontal scrollport, which orphans vertical sticky
        // (wrap has no height constraint, so it never scrolls
        // vertically — and once an element is a scroll container in
        // EITHER axis CSS implicitly promotes the other axis too,
        // capturing all sticky inside it). Letting wrap be a plain
        // div keeps both axes anchored to `_cdcBody`.
        wrap.style.cssText = 'min-width:max-content;';
        this._cdcBody.appendChild(wrap);

        // The overview response carries every mode's matrix in one
        // payload. Pick whichever mode the user is currently viewing
        // (cmdMode echoed by the backend, falling back to the mode-bar
        // selection or the first mode in the dict).
        const modes = (data && data.modes) || {};
        const active = this._cdcActiveModeName(data);
        const modeData = modes[active] || null;

        const allClocks = (modeData && modeData.clocks) || [];
        const matrix = (modeData && modeData.matrix) || {};

        // Per-category totals — folded into the same single pass that
        // computes `activeSet` below to avoid two walks over the
        // matrix. Endpoint_count is the authoritative total because
        // the matrix never carries empty cells (so summing
        // `cell.paths` would double-match), but synced/unsynced/
        // excluded need to be computed here since the backend doesn't
        // emit per-category roll-ups.
        let totalSynced = 0;
        let totalUnsynced = 0;
        let totalExcluded = 0;
        for (const launch of Object.keys(matrix)) {
            const row = matrix[launch] || {};
            for (const capture of Object.keys(row)) {
                const cell = row[capture];
                if (!cell) {
                    continue;
                }
                totalSynced   += +(cell.synced   || 0);
                totalUnsynced += +(cell.unsynced || 0);
                totalExcluded += +(cell.excluded || 0);
            }
        }

        const ep = modeData && modeData.endpoint_count != null
            ? modeData.endpoint_count : 0;
        if (this._cdcStatus) {
            const modeLabel = active ? ` · ${active}` : '';
            // Prefix with the total, then a per-category breakdown so
            // a user scanning the toolbar gets both "how big is this"
            // and "what's the health distribution" in one read. We
            // colourise the unsynced count (red) and synced count
            // (green) inline so the status pill mirrors the matrix
            // legend's traffic-light convention.
            this._cdcStatus.innerHTML
                = `${ep} CDC ${ep === 1 ? 'crossing' : 'crossings'}`
                + (ep > 0
                    ? ' · '
                      + '<span style="color:var(--sdc-text-hold,'
                        + '#ff6b6b);font-weight:600;" '
                        + 'title="paths not synchronized — likely '
                        + 'CDC bugs">'
                      + totalUnsynced + ' unsynced</span>'
                      + ' · '
                      + '<span style="color:var(--sdc-text-setup,'
                        + '#7bc97b);" '
                        + 'title="paths through a recognized '
                        + 'synchronizer (NFF chain, vendor sync '
                        + 'cell, or whitelisted instance/master)">'
                      + totalSynced + ' synced</span>'
                      + ' · '
                      + '<span style="color:var(--fg-muted);" '
                        + 'title="paths excluded by '
                        + 'set_clock_groups or other SDC rules">'
                      + totalExcluded + ' excluded</span>'
                    : '')
                + modeLabel;
        }

        if (allClocks.length === 0) {
            const msg = document.createElement('div');
            msg.style.cssText =
                'padding:16px;color:var(--fg-muted);font-style:italic;';
            msg.textContent = active
                ? `No clocks defined in mode "${active}".`
                : 'No clocks defined in the design.';
            wrap.appendChild(msg);
            return;
        }

        // Compute the set of clocks that actually carry CDC traffic in
        // either direction (launch row OR capture column has any path
        // — across every category, not just unsynced, so synced and
        // excluded crossings stay visible too). On 100+-clock designs
        // most of the matrix is empty, so we hide idle rows/columns by
        // default and let the user opt back in via the "Show all" toggle
        // below the heading. `hasActiveSet` collapses the search:
        // `O(rows × cols)` once over the sparse matrix instead of per
        // render-pass iteration.
        const activeSet = new Set();
        for (const launch of Object.keys(matrix)) {
            const row = matrix[launch] || {};
            for (const capture of Object.keys(row)) {
                const cell = row[capture];
                if (cell && cell.paths > 0) {
                    activeSet.add(launch);
                    activeSet.add(capture);
                }
            }
        }
        // Persist the user's "show all" choice across re-renders within
        // the session — default false so first paint is the focused
        // view. When EVERY clock is active (small designs, demos) the
        // toggle has nothing to hide and we skip rendering it entirely
        // below; the actual filter rule is applied via `clocks` alone.
        if (this._cdcShowAllClocks == null) {
            this._cdcShowAllClocks = false;
        }
        const idleCount = allClocks.length - activeSet.size;
        // Edge case: no clock has any crossings at all. Showing the
        // empty-set "active" view would render an empty 0×0 matrix
        // which is more confusing than just showing every clock with
        // its row of zeros — so we fall back to all-clocks here too.
        const showAll
            = this._cdcShowAllClocks || activeSet.size === 0;
        const clocks = showAll
            ? allClocks
            : allClocks.filter((c) => activeSet.has(c));

        // Single-clock fast-out: a CDC matrix needs at least two clocks
        // to mean anything (the sole cell would always be the diagonal
        // "same clock — not a CDC" entry). Replace it with a clear
        // statement so the user doesn't stare at an empty 1×1 grid
        // wondering whether the scan worked.
        if (allClocks.length === 1) {
            const msg = document.createElement('div');
            msg.style.cssText =
                'padding:16px;color:var(--fg-muted);line-height:1.5;';
            const only = clocks[0];
            msg.innerHTML =
                `Only one clock (<code>${only}</code>) is defined` +
                (active ? ` in mode <code>${active}</code>` : '') +
                ` — no clock-domain crossings to analyse. ` +
                'Define a second clock with <code>create_clock</code> or ' +
                '<code>create_generated_clock</code> to populate the CDC matrix.';
            wrap.appendChild(msg);
            return;
        }

        // Sticky-positioning + z-index constants used by both the
        // banner (heading + toggle) and the matrix headers below.
        // Computed up-front so the banner can reference cornerZ /
        // stickTop / stickLeft without temporal-dead-zone issues.
        // The full ladder is documented above the table-build site
        // below; the short version:
        //   - stickTop / stickLeft compensate for `_cdcBody`'s
        //     padding so cells don't leak through the padding zone
        //     when scrolling.
        //   - cornerZ is the z-index of the top-left corner cell
        //     (above every per-column header).
        //   - Banner z-index sits above cornerZ so it covers the
        //     corner at the top-left intersection.
        const padTop = parseFloat(
            getComputedStyle(this._cdcBody).paddingTop) || 0;
        const padLeft = parseFloat(
            getComputedStyle(this._cdcBody).paddingLeft) || 0;
        const stickTop = -(padTop + 1);
        const stickLeft = -(padLeft + 1);
        const HOVER_BG = 'var(--bg-hover)';
        const HEADER_BG = 'var(--bg-header)';
        const HOVER_FG = 'var(--accent-tab)';
        const COL_Z_BASE = 10;
        const cornerZ = COL_Z_BASE + clocks.length + 1;

        // Sticky banner — the axis-convention heading + idle-clocks
        // toggle row pinned to the top-left of `_cdcBody` so they
        // remain visible no matter how far the user has scrolled
        // into a wide / tall matrix.
        //
        // Width: defaults to its parent's content width (= the
        // table's width because `wrap` is `min-width:max-content`).
        // That's deliberate — when the user scrolls VERTICALLY, the
        // banner pins at the top and must cover EVERY cell that
        // would otherwise scroll up past it, including cells far to
        // the right. A `width:fit-content` banner would only span
        // the heading text, leaving cells to the right of the text
        // visible behind the banner during vertical scroll. Letting
        // the banner extend with the table fixes that. The
        // banner's CONTENT (text + toggle) sits at the left, so when
        // the user scrolls horizontally the banner pins at
        // `left:stickLeft`, the content stays at the viewport's
        // left edge, and the unused right portion scrolls
        // off-screen (cropped by `_cdcBody`'s overflow:auto).
        //
        // z-index sits above every sticky cell because it has to
        // cover the corner cell at the top-left intersection too.
        //
        // The banner's height is measured AFTER it lands in the
        // DOM so the column-header row sticks just BELOW it when
        // both axes pin — the headers can't share `top` with the
        // banner without overlap. jsdom returns 0 from
        // getBoundingClientRect (no real layout), but the sign of
        // `stickTop` and `stickLeft` is what the assertions check
        // — runtime browsers will compute a real height.
        const banner = document.createElement('div');
        banner.style.cssText
            = `position:sticky;top:${stickTop}px;left:${stickLeft}px;`
            + `z-index:${cornerZ + 100};`
            + 'background:var(--bg-main);'
            + 'padding:6px 4px 8px;'
            + 'border-bottom:1px solid var(--border-subtle);';

        const hdr = document.createElement('div');
        hdr.style.cssText
            = 'font-size:12px;color:var(--fg-muted);'
            + 'margin-bottom:4px;';
        const modeHint = Object.keys(modes).length > 1
            ? ` Mode "${active}" — switch the mode selector at the top to flip ` +
              'between cached matrices (no re-scan).'
            : '';
        hdr.textContent =
            'Rows = launch clock, columns = capture clock. ' +
            'Click a cell for the per-path table.' + modeHint;
        banner.appendChild(hdr);

        // Idle-clocks toggle. Only renders when there is something to
        // hide (`idleCount > 0`) AND the matrix isn't already in the
        // empty-set fallback. Re-clicking the toggle flips back to the
        // other view — the button label tracks the current state so
        // users always see what the next click will do.
        if (idleCount > 0 && activeSet.size > 0) {
            const toggleRow = document.createElement('div');
            toggleRow.style.cssText
                = 'font-size:12px;'
                + 'display:flex;align-items:center;gap:8px;';
            const toggleBtn = document.createElement('button');
            toggleBtn.style.cssText
                = 'padding:2px 10px;font-size:12px;cursor:pointer;'
                + 'background:var(--bg-input);color:var(--fg-primary);'
                + 'border:1px solid var(--border);border-radius:3px;';
            toggleBtn.textContent = this._cdcShowAllClocks
                ? `Hide idle clocks (${idleCount})`
                : `Show all clocks (+ ${idleCount} idle)`;
            toggleBtn.title = this._cdcShowAllClocks
                ? 'Hide clocks that have no CDC crossings — collapses '
                  + 'the matrix back to only the clocks that exchange '
                  + 'paths.'
                : 'Reveal every clock in the design, including those '
                  + 'with no CDC crossings. Useful when auditing '
                  + 'whether a clock is missing constraints; otherwise '
                  + 'noisy on large designs.';
            toggleBtn.addEventListener('click', () => {
                this._cdcShowAllClocks = !this._cdcShowAllClocks;
                this._renderCdcMatrix(data);
            });
            toggleRow.appendChild(toggleBtn);
            const status = document.createElement('span');
            status.style.cssText
                = 'color:var(--fg-muted);font-style:italic;';
            status.textContent = this._cdcShowAllClocks
                ? `Showing all ${allClocks.length} clocks `
                  + `(${activeSet.size} with crossings, `
                  + `${idleCount} idle).`
                : `Showing ${activeSet.size} of ${allClocks.length} `
                  + `clocks — ${idleCount} idle clock`
                  + (idleCount === 1 ? '' : 's')
                  + ' hidden (no CDC crossings).';
            toggleRow.appendChild(status);
            banner.appendChild(toggleRow);
        }

        wrap.appendChild(banner);

        // Measure banner height NOW (after DOM insert, before the
        // table is built) so column headers can stick just below it
        // without overlap. Force a synchronous layout via
        // offsetHeight which is cheap and unambiguous.
        const bannerHeight = banner.offsetHeight || 0;
        const colStickTop = stickTop + bannerHeight;

        const tbl = document.createElement('table');
        tbl.style.cssText =
            'border-collapse:collapse;font-size:12px;font-family:monospace;';

        // Header row — column labels rotated -45° so the full clock
        // name fits without inflating column width. Same pattern as the
        // Clock Groups matrix:
        //   - Each header has a fixed width (CELL_W) matching its data
        //     column below, so labels line up.
        //   - Header height (headerPx) is sized from the longest name
        //     so the rotated text fits inside the header band.
        //   - The label is anchored at the column's center-bottom and
        //     rotates about that point, so it grows UP-LEFT into the
        //     empty space above earlier columns instead of overlapping
        //     later columns to the right.
        const thead = document.createElement('thead');
        const hr = document.createElement('tr');
        const CELL_W = 40;
        const longestName = clocks.reduce(
            (m, c) => Math.max(m, c.length), 0);
        // 11px monospace ≈ 6.6px / char; rotated -45° the vertical
        // projection is char_count * 6.6 * sin(45°) ≈ char_count * 4.7.
        // The actual band needs that much plus a hair of breathing room.
        // Earlier this was over-padded (24px slack + min 60px); for the
        // cdc_demo's 5-char clock names that meant a ~60px header for
        // labels that needed ~25px — way too much vertical real estate.
        const headerPx = Math.min(220, Math.max(28, longestName * 5 + 8));

        // Sticky-headers + axis-highlight infrastructure (the orientation
        // affordances for large 100×100-clock matrices):
        //
        //   - The thead row pins to the top edge of `_cdcBody` (the
        //     panel's vertical scroll container) so column labels stay
        //     visible when scrolling deep into a tall matrix. Sticky
        //     bubbles up axes individually: `wrap`'s `overflow-x:auto`
        //     captures horizontal sticky against itself, while vertical
        //     sticky passes through (wrap is overflow-y:visible) to
        //     `_cdcBody`'s overflow-y:auto.
        //   - The leftmost <th> of every body row pins to the left
        //     edge of `wrap` so launch-clock labels stay visible when
        //     scrolling horizontally.
        //   - The corner cell pins to BOTH edges; its z-index has to
        //     beat both axis stripes (see the index ladder below).
        //   - On cell hover, the matching launch-row <th> and capture-
        //     column <th> swap to a `--bg-hover` fill plus bold +
        //     `--accent-tab` text on the rotated span. Cells store
        //     their (row, column) indices closures so the lookup is
        //     O(1) — no DOM walking on every mouseenter.
        //
        // z-index ladder:
        //   COL_Z_BASE + N (decreasing per column) — sticky column
        //     headers, with EARLIER columns sitting above LATER ones
        //     so each rotated label can overflow into the next
        //     column's space without being clipped by the later
        //     column's own opaque header fill. (`position:sticky`
        //     creates a stacking context unconditionally — without
        //     this stagger every <th> would isolate its span and the
        //     next sibling's background would mask the trailing end
        //     of the previous label, the original bug from before
        //     position:sticky was added here.)
        //   2 sticky row <th> (above tbody cells)
        //   1 the rotated <span> inside each column header (relative
        //     to its parent <th>'s own stacking context)
        //   0 tbody cells (default flow)
        // Corner cell sits at COL_Z_BASE + clocks.length + 1 so it
        // beats every column header AND the row labels at the
        // intersection where both axes meet.
        //
        // _cdcBody has padding all around. Sticky `top:0` / `left:0`
        // would pin at the scroll port edge and tbody cells would
        // scroll THROUGH the padding zone, briefly appearing "above"
        // or "to the left" of the headers as they approach the pin
        // position. The negative-offset trick (using the
        // `stickTop`/`stickLeft` constants computed at the top of
        // this function) paper over this — same approach the
        // path-list view already uses.
        const colHeaderEls = [];
        const colHeaderSpans = [];
        const rowHeaderEls = [];

        const corner = document.createElement('th');
        corner.style.cssText =
            `padding:3px 6px;border:1px solid var(--border);` +
            `background:${HEADER_BG};text-align:right;` +
            `height:${headerPx}px;vertical-align:bottom;` +
            `position:sticky;top:${colStickTop}px;left:${stickLeft}px;` +
            `z-index:${cornerZ};` +
            'font-size:11px;color:var(--fg-muted);font-style:italic;';
        corner.innerHTML = 'launch ↓<br>capture →';
        hr.appendChild(corner);
        for (let cIdx = 0; cIdx < clocks.length; ++cIdx) {
            const c = clocks[cIdx];
            const th = document.createElement('th');
            // Stagger z-index: earlier columns higher than later
            // ones, so each header's rotated label paints ABOVE the
            // following column's background. Without the stagger,
            // every column's stacking context would be at the same
            // z and DOM-order would put later columns on top — the
            // user's reported bug ("clock names get cut off by the
            // next column").
            const zCol = COL_Z_BASE + (clocks.length - cIdx);
            th.style.cssText =
                `width:${CELL_W}px;min-width:${CELL_W}px;max-width:${CELL_W}px;` +
                `height:${headerPx}px;padding:0;` +
                'border-bottom:1px solid var(--border);' +
                `background:${HEADER_BG};` +
                `vertical-align:bottom;position:sticky;top:${colStickTop}px;` +
                `z-index:${zCol};`;
            // Anchor span's bottom-left at the column's center-bottom
            // (left:50%, bottom:2px) and rotate -45° about that
            // corner. The text grows up-and-to-the-right from there.
            //
            // z-index:1 is load-bearing: long clock names extend
            // horizontally past the column they belong to, into the
            // area where adjacent <th> cells live. Each <th> has its
            // own opaque header-row background that paints AFTER the
            // previous label (DOM order), so without an explicit
            // z-index the next column's fill masks the trailing end
            // of the previous label. Bumping the label into a
            // higher stacking layer keeps it above sibling
            // backgrounds.
            const span = document.createElement('span');
            span.style.cssText =
                'position:absolute;left:50%;bottom:2px;z-index:1;' +
                'transform-origin:left bottom;' +
                'transform:rotate(-45deg);' +
                'white-space:nowrap;' +
                'font-size:12px;color:var(--fg-primary);' +
                'transition:color 80ms;';
            span.textContent = c;
            span.title = c;
            th.appendChild(span);
            hr.appendChild(th);
            colHeaderEls.push(th);
            colHeaderSpans.push(span);
        }
        thead.appendChild(hr);
        tbl.appendChild(thead);

        // Toggle the axis-highlight on the row's <th> + the column's
        // <th>+<span> for a single cell. Pulled out so mouseenter and
        // mouseleave use the exact same set of mutations (with `on`
        // flipping bg + weight + colour).
        const setAxisHover = (rowIdx, colIdx, on) => {
            const rowEl = rowHeaderEls[rowIdx];
            const colEl = colHeaderEls[colIdx];
            const colSpan = colHeaderSpans[colIdx];
            if (rowEl) {
                rowEl.style.background = on ? HOVER_BG : HEADER_BG;
                rowEl.style.fontWeight = on ? '700' : '';
                // Match the column header's text-color flip on hover —
                // the row label was previously left at default fg
                // while the column label flipped to HOVER_FG, and the
                // asymmetric visual was confusing.
                rowEl.style.color = on ? HOVER_FG : 'var(--fg-primary)';
            }
            if (colEl) {
                colEl.style.background = on ? HOVER_BG : HEADER_BG;
            }
            if (colSpan) {
                colSpan.style.color = on ? HOVER_FG : 'var(--fg-primary)';
                colSpan.style.fontWeight = on ? '700' : '';
            }
        };

        // Body
        const tbody = document.createElement('tbody');
        for (let r = 0; r < clocks.length; ++r) {
            const launch = clocks[r];
            const row = document.createElement('tr');
            const lbl = document.createElement('th');
            // `position:sticky;left:stickLeft` pins the row label to
            // the left edge of `_cdcBody` (the scroll container);
            // negative offset compensates for the container's
            // padding-left so cells don't leak through.
            lbl.style.cssText =
                'padding:4px 8px;border:1px solid var(--border);' +
                `background:${HEADER_BG};text-align:right;` +
                `position:sticky;left:${stickLeft}px;z-index:2;`;
            lbl.textContent = launch;
            row.appendChild(lbl);
            rowHeaderEls.push(lbl);

            for (let cIdx = 0; cIdx < clocks.length; ++cIdx) {
                const capture = clocks[cIdx];
                const td = document.createElement('td');
                td.style.cssText =
                    'padding:4px 0;border:1px solid var(--border);' +
                    `text-align:center;` +
                    `width:${CELL_W}px;min-width:${CELL_W}px;` +
                    `max-width:${CELL_W}px;`;
                // Every cell — including the diagonal "—" and the
                // empty "0" — participates in the hover affordance so
                // the user can use them as orientation cues even when
                // they don't carry traffic.
                td.addEventListener('mouseenter',
                    () => setAxisHover(r, cIdx, true));
                td.addEventListener('mouseleave',
                    () => setAxisHover(r, cIdx, false));
                if (launch === capture) {
                    td.textContent = '—';
                    td.style.background = HEADER_BG;
                    td.style.color = 'var(--fg-muted)';
                    td.title = `${launch} → ${capture} — same clock, not a CDC`;
                    row.appendChild(td);
                    continue;
                }
                const cell = (matrix[launch] && matrix[launch][capture]) || null;
                if (!cell || cell.paths === 0) {
                    td.textContent = '0';
                    td.style.color = 'var(--fg-muted)';
                    td.title = `${launch} → ${capture} — no CDC paths`;
                    row.appendChild(td);
                    continue;
                }
                td.textContent = String(cell.paths);
                // Traffic-light fill (semi-transparent so dark/light themes
                // both stay readable).
                let bg, fg, hint;
                if (cell.unsynced > 0) {
                    bg  = 'rgba(220, 64, 64, 0.30)';
                    fg  = 'var(--fg-primary)';
                    hint = 'unsynced paths present';
                } else if (cell.synced > 0) {
                    bg  = 'rgba(76, 175, 80, 0.25)';
                    fg  = 'var(--fg-primary)';
                    hint = 'every path synchronized or excluded';
                } else {
                    bg  = 'rgba(150, 150, 150, 0.18)';
                    fg  = 'var(--fg-secondary)';
                    hint = 'every path excluded by SDC';
                }
                td.style.background = bg;
                td.style.color = fg;
                td.style.cursor = 'pointer';
                td.style.fontWeight = '600';
                td.title =
                    `${launch} → ${capture}\n` +
                    `${cell.paths} total · ${cell.synced} synced · ` +
                    `${cell.excluded} excluded · ${cell.unsynced} unsynced — ` +
                    hint + '. Click for the per-path table.';
                td.addEventListener('click', () =>
                    this._loadCdcPaths(launch, capture));
                td.dataset.cdcLaunch  = launch;
                td.dataset.cdcCapture = capture;
                row.appendChild(td);
            }
            tbody.appendChild(row);
        }
        tbl.appendChild(tbody);
        wrap.appendChild(tbl);

        // Legend.
        const legend = document.createElement('div');
        legend.style.cssText =
            'padding:12px 4px 0;font-size:12px;color:var(--fg-muted);';
        legend.innerHTML =
            '<span style="display:inline-block;width:10px;height:10px;' +
            'background:rgba(220,64,64,0.6);margin-right:4px;"></span>' +
            'has unsynced paths (action needed) &nbsp; ' +
            '<span style="display:inline-block;width:10px;height:10px;' +
            'background:rgba(76,175,80,0.6);margin-right:4px;"></span>' +
            'all paths synchronized &nbsp; ' +
            '<span style="display:inline-block;width:10px;height:10px;' +
            'background:rgba(150,150,150,0.5);margin-right:4px;"></span>' +
            'all paths excluded by SDC';
        wrap.appendChild(legend);

        // Mix-endpoint listing — two collapsible sections that surface
        // every endpoint where ≥2 clocks reach a single pin, grouped
        // by where the mix originates. Lazy-fetched on first expand.
        // Renders below the matrix because that's where the user lands
        // after reading the matrix totals and asks "which endpoints?".
        if (allClocks.length >= 2) {
            this._renderMixEndpointsSections(wrap, active);
        }
    }

    // Build the two mix-endpoint sections (data-path + clock-path) and
    // append them to `parent`. Both sections start collapsed; clicking
    // a header lazy-fetches `cdc_mix_endpoints` once and caches the
    // result for the session (re-fetched only when the active mode
    // changes or _renderCdcMatrix is invoked with a fresh overview).
    //
    // The cache lives on `this._cdcMixEndpointsCache` keyed by mode
    // name; both sections of a given mode share the same fetch result
    // (the RPC returns both buckets in one call).
    _renderMixEndpointsSections(parent, modeName) {
        const wrap = document.createElement('div');
        wrap.dataset.cdcMixEndpoints = '1';
        wrap.style.cssText
            = 'display:flex;flex-direction:column;gap:8px;'
            + 'margin-top:14px;font-size:12px;';
        parent.appendChild(wrap);

        // Filter state — shared across both sections so a single Kind
        // chip toggle filters everywhere. Sync-status only applies to
        // the data-path bucket (clock-path mix has no synchronizer
        // semantic). Default to "unsynchronized" because that's the bug
        // list the user is actually triaging.
        const state = {
            mode: modeName || '',
            sync: 'unsynchronized',
            kinds: new Set(),  // empty = all kinds
        };
        // Stash the state on `this` so re-renders (mode changes, kind
        // toggles) can mutate the same object without re-creating the
        // section DOM.
        this._cdcMixEndpointsState = state;

        const dataSection = this._buildMixEndpointSection({
            id: 'data',
            label: 'Data-path mix endpoints',
            includeSync: true,
            state,
        });
        const clockSection = this._buildMixEndpointSection({
            id: 'clock',
            label: 'Clock-path mix endpoints',
            includeSync: false,
            state,
        });
        wrap.appendChild(dataSection.host);
        wrap.appendChild(clockSection.host);
        // Both sections share the same fetch promise; refreshing one
        // re-renders both because filter changes affect the counts on
        // both.
        state.refreshAll = () => {
            dataSection.refresh();
            clockSection.refresh();
        };
    }

    // Construct one collapsible section (host + header + body) and
    // return refs the caller can use to expand/refresh.
    _buildMixEndpointSection({id, label, includeSync, state}) {
        const host = document.createElement('div');
        host.dataset.cdcMixSection = id;
        host.style.cssText
            = 'border:1px solid var(--border);border-radius:3px;'
            + 'background:var(--bg-secondary);';

        const header = document.createElement('div');
        header.style.cssText
            = 'display:flex;align-items:center;gap:8px;'
            + 'padding:6px 10px;cursor:pointer;'
            + 'font-weight:600;color:var(--fg-primary);'
            + 'user-select:none;';
        header.title = id === 'data'
            ? 'Endpoints whose D pin sees ≥2 clocks (data-path mix). '
              + 'Click to load and group by where the mix originates.'
            : 'Flops whose CK pin sees ≥2 clocks (clock-path mix). '
              + 'Click to load and group by where the clock-OR mixes.';
        const arrow = document.createElement('span');
        arrow.textContent = '▸';
        arrow.style.cssText = 'display:inline-block;width:10px;'
            + 'transition:transform 120ms;';
        const title = document.createElement('span');
        title.textContent = label;
        const counts = document.createElement('span');
        counts.style.cssText = 'color:var(--fg-muted);font-weight:400;';
        counts.textContent = '(— click to compute)';
        header.appendChild(arrow);
        header.appendChild(title);
        header.appendChild(counts);
        host.appendChild(header);

        const body = document.createElement('div');
        body.style.cssText = 'display:none;padding:8px 10px 10px;'
            + 'border-top:1px solid var(--border);';
        host.appendChild(body);

        let expanded = false;
        let loaded = false;
        let loading = false;

        const refresh = () => {
            if (!loaded || !expanded) {
                return;
            }
            this._refreshMixEndpointSection({
                bodyEl: body,
                headerCounts: counts,
                bucketName: id === 'data' ? 'data_path' : 'clock_path',
                includeSync,
                state,
            });
        };

        const expand = async () => {
            expanded = true;
            arrow.style.transform = 'rotate(90deg)';
            body.style.display = 'block';
            if (loaded) {
                refresh();
                return;
            }
            if (loading) {
                return;
            }
            loading = true;
            counts.textContent = '(loading…)';
            try {
                await this._fetchCdcMixEndpoints(state.mode);
                loaded = true;
                refresh();
            } catch (e) {
                console.warn('[CDC] mix-endpoints fetch failed', e);
                counts.textContent = '(load failed — click to retry)';
                loading = false;
            }
            loading = false;
        };
        const collapse = () => {
            expanded = false;
            arrow.style.transform = '';
            body.style.display = 'none';
        };
        header.addEventListener('click', () => {
            if (expanded) {
                collapse();
            } else {
                expand();
            }
        });

        return {host, refresh};
    }

    // Lazy fetch + cache. The promise is shared across both sections
    // and across the filter-driven re-renders.
    async _fetchCdcMixEndpoints(modeName) {
        if (!this._cdcMixEndpointsCache) {
            this._cdcMixEndpointsCache = new Map();
        }
        const key = modeName || '';
        if (this._cdcMixEndpointsCache.has(key)) {
            return this._cdcMixEndpointsCache.get(key);
        }
        await this._app.websocketManager.readyPromise;
        const promise = this._requestWithTimeout({
            type: 'cdc_mix_endpoints',
            mode: modeName,
        });
        // Cache the promise so two concurrent expand calls share one
        // RPC. Replace with the resolved data after it lands so later
        // accesses are sync.
        this._cdcMixEndpointsCache.set(key, promise);
        try {
            const data = await promise;
            this._cdcMixEndpointsCache.set(key, data);
            return data;
        } catch (e) {
            this._cdcMixEndpointsCache.delete(key);
            throw e;
        }
    }

    // Render (or re-render) a section body from the cached payload,
    // applying current filters and updating the section header counts.
    _refreshMixEndpointSection({
        bodyEl, headerCounts, bucketName, includeSync, state
    }) {
        const cached = this._cdcMixEndpointsCache
            ? this._cdcMixEndpointsCache.get(state.mode || '') : null;
        if (!cached || cached.then) {
            // Still resolving (we shouldn't be here); skip.
            return;
        }
        const bucket = (cached && cached[bucketName]) || {
            groups: [], total_endpoints: 0, total_origins: 0,
            kinds_total: {}, sync_total: {},
        };
        bodyEl.innerHTML = '';

        // ── Section description ──
        // Each bucket shows a short explainer so the meaning of a row +
        // an expanded card is self-evident. The user shouldn't have
        // to read source to figure out what "origin" or "endpoint"
        // refers to in the listing.
        const desc = document.createElement('div');
        desc.style.cssText
            = 'margin-bottom:8px;color:var(--fg-muted);'
            + 'font-size:11px;line-height:1.4;';
        if (bucketName === 'data_path') {
            desc.textContent
                = 'Each card is a gate where two or more clock domains '
                + 'converge on a data path. Expand to see the capture '
                + 'flops downstream of that gate (where the mixed data '
                + 'is sampled). "Unsynced" rows are likely CDC bugs.';
        } else {
            desc.textContent
                = 'Each card is a gate where two or more clocks '
                + 'converge on a clock-network path. Expand to see '
                + 'the flops whose CK pin is fed by that mix — they '
                + 'launch for multiple capture domains at once.';
        }
        bodyEl.appendChild(desc);

        // ── Filter bar ──
        // Always-visible kind chips on top; sync-status radio only on
        // the data-path bucket.
        const filterBar = document.createElement('div');
        filterBar.style.cssText
            = 'display:flex;flex-wrap:wrap;align-items:center;'
            + 'gap:8px 12px;margin-bottom:8px;';
        if (includeSync) {
            filterBar.appendChild(
                this._buildMixSyncRadio(state, bucket.sync_total || {}));
        }
        filterBar.appendChild(
            this._buildMixKindChips(state, bucket.kinds_total || {}));
        bodyEl.appendChild(filterBar);

        // ── Filter the cached data ──
        const filtered = this._filterMixBucket(bucket, includeSync, state);

        // ── Header counts ──
        // Data-path bucket counts STA endpoints (one per D-pin); the
        // clock-path bucket dedupes to one row per flop instance, so
        // the unit there is "flops" not "endpoints".
        const groupCount = filtered.groups.length;
        const epCount = filtered.totalEndpoints;
        const epWord = bucketName === 'clock_path' ? 'flop' : 'endpoint';
        const epWordPl = bucketName === 'clock_path' ? 'flops' : 'endpoints';
        if (epCount === 0) {
            headerCounts.textContent
                = bucket.total_endpoints === 0
                    ? `(no multi-clock ${epWordPl})`
                    : `(0 ${epWordPl} match filters)`;
        } else {
            headerCounts.textContent
                = `(${epCount} ${epCount === 1 ? epWord : epWordPl} `
                + `from ${groupCount} `
                + `${groupCount === 1 ? 'origin' : 'origins'})`;
        }

        // ── Render groups ──
        if (filtered.groups.length === 0) {
            const empty = document.createElement('div');
            empty.style.cssText
                = 'padding:8px;color:var(--fg-muted);'
                + 'font-style:italic;';
            empty.textContent = bucket.total_endpoints === 0
                ? 'No endpoints in this bucket.'
                : 'No endpoints match the active filters.';
            bodyEl.appendChild(empty);
            return;
        }
        for (const grp of filtered.groups) {
            const ec = (grp.endpoints || []).length;
            grp._unitWord = ec === 1 ? epWord : epWordPl;
            bodyEl.appendChild(
                this._buildMixOriginGroup(grp, includeSync));
        }
    }

    // Sync-status radio (data-path only). Counts reflect the unfiltered
    // bucket so the user can see how many endpoints exist in each
    // category before narrowing.
    _buildMixSyncRadio(state, syncTotal) {
        const wrap = document.createElement('div');
        wrap.style.cssText
            = 'display:inline-flex;align-items:center;gap:6px;';
        const labelEl = document.createElement('span');
        labelEl.textContent = 'Sync:';
        labelEl.style.cssText = 'color:var(--fg-muted);';
        wrap.appendChild(labelEl);
        const opts = [
            {key: 'unsynchronized',
             label: 'unsynchronized',
             count: this._mixSyncUnsyncedCount(syncTotal)},
            {key: 'all', label: 'all', count: this._mixSyncTotal(syncTotal)},
            {key: 'synced',
             label: 'synced',
             count: this._mixSyncSyncedCount(syncTotal)},
        ];
        for (const o of opts) {
            const btn = document.createElement('span');
            const selected = state.sync === o.key;
            btn.style.cssText
                = 'padding:2px 8px;border-radius:10px;cursor:pointer;'
                + 'border:1px solid var(--border);'
                + (selected
                    ? 'background:var(--accent-tab);color:var(--bg-primary);'
                    : 'background:var(--bg-primary);color:var(--fg-primary);');
            btn.textContent = `${o.label} (${o.count})`;
            btn.addEventListener('click', () => {
                if (state.sync !== o.key) {
                    state.sync = o.key;
                    state.refreshAll();
                }
            });
            wrap.appendChild(btn);
        }
        return wrap;
    }

    _mixSyncUnsyncedCount(syncTotal) {
        // "unsynchronized" = chain detection found no synchronizer.
        return +(syncTotal['none'] || 0);
    }
    _mixSyncSyncedCount(syncTotal) {
        return +(syncTotal['ff_chain'] || 0)
             + +(syncTotal['liberty_sync'] || 0)
             + +(syncTotal['composite'] || 0)
             + +(syncTotal['whitelisted'] || 0);
    }
    _mixSyncTotal(syncTotal) {
        let t = 0;
        for (const k of Object.keys(syncTotal || {})) {
            t += +(syncTotal[k] || 0);
        }
        return t;
    }

    // Endpoint-kind chip filter — same convention as the SDC Endpoints
    // tab. An empty selection means "all kinds". Counts reflect the
    // unfiltered bucket.
    _buildMixKindChips(state, kindsTotal) {
        const wrap = document.createElement('div');
        wrap.style.cssText
            = 'display:inline-flex;align-items:center;gap:6px;'
            + 'flex-wrap:wrap;';
        const labelEl = document.createElement('span');
        labelEl.textContent = 'Kind:';
        labelEl.style.cssText = 'color:var(--fg-muted);';
        wrap.appendChild(labelEl);
        const order = ['flipflop', 'latch', 'port', 'macro', 'clock_gate',
                       'stdcell'];
        const labelMap = {
            flipflop: 'FF', latch: 'latch', port: 'port',
            macro: 'macro', clock_gate: 'ICG', stdcell: 'stdcell',
        };
        for (const k of order) {
            const cnt = +(kindsTotal[k] || 0);
            if (cnt === 0) {
                continue;
            }
            const chip = document.createElement('span');
            const selected = state.kinds.has(k);
            chip.style.cssText
                = 'padding:2px 8px;border-radius:10px;cursor:pointer;'
                + 'border:1px solid var(--border);'
                + (selected
                    ? 'background:var(--accent-tab);color:var(--bg-primary);'
                    : 'background:var(--bg-primary);color:var(--fg-primary);');
            chip.textContent = `${labelMap[k] || k} (${cnt})`;
            chip.addEventListener('click', () => {
                if (state.kinds.has(k)) {
                    state.kinds.delete(k);
                } else {
                    state.kinds.add(k);
                }
                state.refreshAll();
            });
            wrap.appendChild(chip);
        }
        return wrap;
    }

    // Apply the active filters to a bucket and return the visible
    // groups + total endpoint count. Groups with zero matching
    // endpoints after filtering are dropped entirely.
    _filterMixBucket(bucket, includeSync, state) {
        const groups = (bucket && bucket.groups) || [];
        const out = [];
        let totalEndpoints = 0;
        for (const g of groups) {
            const eps = (g.endpoints || []).filter(e => {
                if (state.kinds.size > 0 && !state.kinds.has(e.kind)) {
                    return false;
                }
                if (includeSync && state.sync !== 'all') {
                    const k = (e.sync_status && e.sync_status.kind) || 'none';
                    if (state.sync === 'unsynchronized' && k !== 'none') {
                        return false;
                    }
                    if (state.sync === 'synced' && k === 'none') {
                        return false;
                    }
                }
                return true;
            });
            if (eps.length === 0) {
                continue;
            }
            out.push({...g, endpoints: eps});
            totalEndpoints += eps.length;
        }
        return {groups: out, totalEndpoints};
    }

    // One mix-origin group: header (origin label + clock-set + count)
    // collapses an endpoint table.
    _buildMixOriginGroup(grp, includeSync) {
        const host = document.createElement('div');
        host.style.cssText
            = 'border:1px solid var(--border-faint, var(--border));'
            + 'border-radius:3px;margin-top:6px;'
            + 'background:var(--bg-primary);';

        const header = document.createElement('div');
        header.style.cssText
            = 'display:flex;align-items:center;gap:8px;'
            + 'padding:5px 8px;cursor:pointer;user-select:none;';
        const arrow = document.createElement('span');
        arrow.textContent = '▸';
        arrow.style.cssText = 'display:inline-block;width:10px;'
            + 'color:var(--fg-muted);transition:transform 120ms;';
        header.appendChild(arrow);

        // Origin label: short last-segment by default, full path in
        // tooltip. Long hierarchical paths leading-ellipsis-truncate
        // via the same TRUNCATE_PATH_CSS the trace-mix cards use, so
        // the tail (the actual gate name) stays visible. The title
        // lives on the OUTER wrapper, not the rtl-truncated inner
        // span — browsers inherit the element's `direction` property
        // into the native tooltip and a tooltip on a `direction:rtl`
        // element renders right-aligned, which is what the user
        // reported as misaligned.
        const originLblWrap = document.createElement('span');
        originLblWrap.style.cssText
            = 'flex:0 1 auto;min-width:0;'
            + 'display:flex;align-items:baseline;';
        originLblWrap.title = this._mixOriginTooltip(grp);
        const originLbl = document.createElement('span');
        originLbl.style.cssText
            = 'font-family:monospace;flex:0 1 auto;min-width:0;'
            + TRUNCATE_PATH_CSS;
        const labelText = this._mixOriginLabel(grp);
        originLbl.textContent = labelText;
        originLblWrap.appendChild(originLbl);
        header.appendChild(originLblWrap);

        const clocksLbl = document.createElement('span');
        clocksLbl.style.cssText
            = 'display:inline-flex;align-items:center;gap:4px;'
            + 'flex:0 0 auto;';
        for (const c of (grp.clocks || [])) {
            clocksLbl.appendChild(this._cdcMakeClockChip(c));
        }
        header.appendChild(clocksLbl);

        const cntLbl = document.createElement('span');
        cntLbl.style.cssText
            = 'margin-left:auto;color:var(--fg-muted);'
            + 'flex:0 0 auto;';
        const ec = (grp.endpoints || []).length;
        // Word matches the section's unit (set on the host by the
        // section builder) so the per-group label tracks the per-
        // section header.
        const word = grp._unitWord || (ec === 1 ? 'endpoint' : 'endpoints');
        cntLbl.textContent = `${ec} ${word}`;
        header.appendChild(cntLbl);
        host.appendChild(header);

        const body = document.createElement('div');
        body.style.cssText = 'display:none;padding:6px 8px 6px 22px;'
            + 'border-top:1px solid var(--border-faint, var(--border));';
        host.appendChild(body);

        let expanded = false;
        header.addEventListener('click', () => {
            expanded = !expanded;
            arrow.style.transform = expanded ? 'rotate(90deg)' : '';
            body.style.display = expanded ? 'block' : 'none';
            if (expanded && body.children.length === 0) {
                this._renderMixGroupEndpoints(body, grp, includeSync);
            }
        });
        return host;
    }

    // Origin label rules:
    //   Mixer / NetMixer  → "<inst>/<pin>"  (the gate/net mixing clocks)
    //   Port              → "port: <pin>"   (multi-clock arrives via SDC)
    //   DepthLimit / Feedback / Stuck / Unresolved / NoOrigin →
    //     short "(<reason>)" placeholder. The pin/inst these terminals
    //     carried was where ONE endpoint's walk gave up — not the group
    //     identity — so the backend collapses them by (kind, clocks)
    //     and we show only the kind here.
    _mixOriginLabel(grp) {
        const k = grp.origin_kind;
        const inst = grp.origin_inst || '';
        const pin = grp.origin_pin || '';
        if (k === 'mixer' || k === 'net_mixer') {
            // For net_mixer the origin "pin" is the load-side pin (the
            // net itself is the mixer), so showing the load pin is the
            // most informative label.
            return pin || inst || '(unknown origin)';
        }
        if (k === 'port') {
            return `port: ${pin || '(unknown)'}`;
        }
        if (k === 'depth_limit') return '(walk depth limit)';
        if (k === 'feedback') return '(feedback loop)';
        if (k === 'stuck') return '(stuck — no upstream contributor)';
        return '(no origin found)';
    }
    // Per-kind tooltip explanation. Surfaces what each terminal kind
    // actually means so users don't have to read source to understand
    // the listing — particularly important for the "(feedback)" /
    // "(walk depth limit)" cases that look cryptic at first.
    _mixOriginKindHelp(k) {
        switch (k) {
            case 'mixer':
                return 'A combinational gate where two or more clock '
                     + 'domains converge. This is the typical CDC '
                     + 'origin — fix it with a synchronizer or by '
                     + 'narrowing the data path to a single domain.';
            case 'net_mixer':
                return 'A net with multiple drivers, each on a '
                     + 'different clock. Treated like a comb-cell '
                     + 'mixer; fix at the driver level.';
            case 'port':
                return 'Multi-clock data arrives via a top-level '
                     + 'port (set_input_delay attaches multiple '
                     + 'clocks). Fix in SDC or with a domain-aware '
                     + 'wrapper.';
            case 'depth_limit':
                return 'The mix-origin walk reached its depth cap '
                     + '(20 stages) without finding a converging '
                     + 'gate. Likely a deep clock tree or buffered '
                     + 'data path; the actual mix is further '
                     + 'upstream. Open the per-row trace to walk '
                     + 'further.';
            case 'feedback':
                return 'The walk hit a cycle (combinational loop or '
                     + 'feedback path) before finding a converging '
                     + 'gate. Often benign in clock-mux structures; '
                     + 'inspect the cycle to confirm.';
            case 'stuck':
                return 'No input on the gate intersected the clocks '
                     + 'we were tracking — phantom propagation. '
                     + 'Usually a sign that STA propagated clocks '
                     + 'across a constant or disabled arc.';
            case 'unresolved':
            case 'no_origin':
                return 'The walk could not find a driver for this '
                     + 'net (undefined boundary, blackbox, or '
                     + 'partially-linked design). The mix is real '
                     + 'but the origin is not resolvable from the '
                     + 'current netlist.';
        }
        return '';
    }
    _mixOriginTooltip(grp) {
        const lbl = this._mixOriginLabel(grp);
        const full = grp.origin_pin_full || grp.origin_inst_full || '';
        const clk = (grp.clocks || []).join(', ');
        const lines = [lbl];
        if (full && full !== lbl) {
            lines.push(full);
        }
        if (clk) {
            lines.push(`clocks: ${clk}`);
        }
        const help = this._mixOriginKindHelp(grp.origin_kind);
        if (help) {
            lines.push('');  // blank separator
            lines.push(help);
        }
        return lines.join('\n');
    }

    _renderMixGroupEndpoints(parent, grp, includeSync) {
        const eps = grp.endpoints || [];
        for (const e of eps) {
            const row = document.createElement('div');
            row.style.cssText
                = 'display:flex;align-items:center;gap:8px;'
                + 'padding:3px 0;font-size:12px;';
            // Outer wrapper holds the `title` so the native tooltip
            // doesn't inherit `direction:rtl` from the truncated inner
            // span and end up right-aligned.
            const nameWrap = document.createElement('span');
            nameWrap.style.cssText
                = 'flex:1 1 auto;min-width:0;display:flex;'
                + 'align-items:baseline;';
            nameWrap.title = e.name;
            const nameEl = document.createElement('span');
            nameEl.style.cssText
                = 'font-family:monospace;flex:1 1 auto;min-width:0;'
                + TRUNCATE_PATH_CSS;
            nameEl.textContent = e.name;
            // _linkifyPin makes the pin name clickable into the
            // inspector. The pin reference shape it expects matches
            // {odb_type, odb_id, name} — emit endpoints carry exactly
            // those fields plus 'kind' / 'clocks' / 'sync_status'.
            this._linkifyPin(nameEl, {
                odb_type: e.pin_odb_type,
                odb_id: e.pin_odb_id,
                name: e.name,
            }, 'name');
            nameWrap.appendChild(nameEl);
            row.appendChild(nameWrap);

            const kindEl = document.createElement('span');
            kindEl.style.cssText
                = 'color:var(--fg-muted);flex:0 0 auto;font-size:11px;';
            kindEl.textContent = e.kind;
            row.appendChild(kindEl);

            const clocksEl = document.createElement('span');
            clocksEl.style.cssText
                = 'display:inline-flex;align-items:center;gap:3px;'
                + 'flex:0 0 auto;';
            for (const c of (e.clocks || [])) {
                clocksEl.appendChild(this._cdcMakeClockChip(c));
            }
            row.appendChild(clocksEl);

            // Trace-mix button — opens the existing convergence-tree
            // tracer, anchored to this row (placement:'after' so the
            // trace appears below the row as an in-place expansion).
            // Available on every row regardless of bucket; the trace
            // walk works the same on D pins (data-path) and CK pins
            // (clock-path) thanks to `trace_pin_odb_*` always pointing
            // at the right pin.
            if (e.trace_pin_odb_type != null && e.trace_pin_odb_id != null
                && Array.isArray(e.clocks) && e.clocks.length > 1) {
                const traceBtn = document.createElement('a');
                traceBtn.textContent = '↑ trace';
                traceBtn.style.cssText
                    = 'flex:0 0 auto;font-size:11px;cursor:pointer;'
                    + 'color:var(--accent-tab);text-decoration:underline;';
                traceBtn.title
                    = 'Walk upstream and show where the multi-clock '
                    + 'mix converges (toggleable).';
                const tracePinRef = {
                    odb_type: e.trace_pin_odb_type,
                    odb_id: e.trace_pin_odb_id,
                };
                const traceClocks = e.clocks.slice();
                const tracePinKind = includeSync ? 'data' : 'clock';
                traceBtn.addEventListener('click', (ev) => {
                    ev.preventDefault();
                    ev.stopPropagation();
                    this._toggleCdcClockMix(
                        tracePinRef, traceClocks, traceBtn,
                        tracePinKind, e.name,
                        {anchor: row, placement: 'after'});
                });
                row.appendChild(traceBtn);
            }

            // Reveal-in-matrix link — only meaningful for register
            // endpoints (where capture_clocks landed on the entry).
            // Top-level output ports / macros / stdcells skip the
            // affordance because they have no (launch, capture) cell
            // in the matrix to jump to.
            if (Array.isArray(e.capture_clocks)
                && e.capture_clocks.length > 0
                && Array.isArray(e.clocks)
                && e.clocks.length > 0) {
                const reveal = document.createElement('a');
                reveal.textContent = '↘ matrix';
                reveal.style.cssText
                    = 'flex:0 0 auto;font-size:11px;cursor:pointer;'
                    + 'color:var(--accent-tab);text-decoration:underline;';
                const pairs = [];
                for (const launch of e.clocks) {
                    for (const capture of e.capture_clocks) {
                        pairs.push([launch, capture]);
                    }
                }
                reveal.title
                    = `Highlight matrix cell${pairs.length === 1 ? '' : 's'}: `
                    + pairs.map(([l, c]) => `${l} → ${c}`).join(', ');
                reveal.addEventListener('click', (ev) => {
                    ev.preventDefault();
                    ev.stopPropagation();
                    this._revealCdcMatrixCells(pairs);
                });
                row.appendChild(reveal);
            }

            if (includeSync && e.sync_status) {
                const syncEl = document.createElement('span');
                const k = e.sync_status.kind || 'none';
                const isUnsynced = k === 'none';
                syncEl.textContent = isUnsynced
                    ? 'unsynced'
                    : (k === 'whitelisted'
                        ? 'whitelisted'
                        : `${k} (depth ${e.sync_status.depth || 0})`);
                syncEl.style.cssText
                    = 'flex:0 0 auto;font-size:11px;'
                    + (isUnsynced
                        ? 'color:var(--sdc-text-hold,#ff6b6b);font-weight:600;'
                        : 'color:var(--fg-muted);');
                row.appendChild(syncEl);
            }

            parent.appendChild(row);
        }
    }

    // Scroll the CDC matrix into view and flash-highlight the cells
    // at the given (launch, capture) pairs. Used by the mix-endpoint
    // listing's `↘ matrix` link to jump from "this endpoint mixes
    // {clk_a, clk_b} on its D, captured by clk_b" to the matrix
    // cells (clk_a, clk_b) and (clk_b, clk_b).
    //
    // Cells are tagged with `data-cdc-launch` / `data-cdc-capture`
    // by _renderCdcMatrix, so the lookup is a single querySelector
    // per pair. Missing cells (sparse storage when paths == 0) are
    // ignored silently — the matrix only renders cells that have
    // crossings, and a (launch, capture) pair the user wants to
    // reveal might not exist if STA didn't see a path.
    _revealCdcMatrixCells(pairs) {
        if (!pairs || !pairs.length || !this._cdcBody) {
            return;
        }
        // Scroll the matrix into view first so the flash is actually
        // visible. _cdcBody is the panel's scroll container; scroll
        // its first cell match into view (smooth scrolling so the
        // motion is a clear visual cue, not an instant jump).
        const cells = [];
        for (const [launch, capture] of pairs) {
            const sel = `[data-cdc-launch="${CSS.escape(launch)}"]`
                + `[data-cdc-capture="${CSS.escape(capture)}"]`;
            const el = this._cdcBody.querySelector(sel);
            if (el) {
                cells.push(el);
            }
        }
        if (cells.length === 0) {
            return;
        }
        cells[0].scrollIntoView(
            {behavior: 'smooth', block: 'center', inline: 'center'});

        // Two-stage flash: ring overlay (outline) on every matched
        // cell so the user can spot multiple matches at a glance.
        // Outline doesn't affect layout (unlike border), so the
        // matrix doesn't shift while the highlight is on.
        const RING_MS = 1400;
        for (const cell of cells) {
            const prevOutline = cell.style.outline;
            const prevOutlineOffset = cell.style.outlineOffset;
            cell.style.outline = '2px solid var(--accent-tab)';
            cell.style.outlineOffset = '-2px';
            setTimeout(() => {
                cell.style.outline = prevOutline;
                cell.style.outlineOffset = prevOutlineOffset;
            }, RING_MS);
        }
    }

    // Fetch the first page of paths for a (launch, capture) cell and
    // render the table. Subsequent pages stream in via the scroll
    // handler installed in _renderCdcPaths and the post-render
    // top-up. Backend memoises the heavy walk per mode (see
    // PairCache in cdc_handler.cpp), so subsequent page requests
    // and category-filter switches are cache hits.
    async _loadCdcPaths(launch, capture, opts) {
        opts = opts || {};
        const category = opts.category || 'all';
        // `opts.pattern` is the user-typed glob/substring; we fold a
        // bare word (no `*`/`?`) into `*word*` here so backend
        // sta::PatternMatch lands on the same substring affordance
        // the SDC tab's other glob inputs offer. Empty / whitespace
        // → no narrowing.
        const rawPattern = (opts.pattern != null
            ? opts.pattern
            : (this._cdcCurrentPattern || '')).trim();
        const pattern = rawPattern && !/[*?]/.test(rawPattern)
            ? `*${rawPattern}*`
            : rawPattern;
        this._cdcCurrentLaunch  = launch;
        this._cdcCurrentCapture = capture;
        this._cdcCurrentCategoryFilter = category;
        this._cdcCurrentPattern = rawPattern;
        // Reset pagination state for a fresh query.
        this._cdcPathsLoaded   = 0;
        this._cdcPathsTotal    = 0;
        this._cdcPathsFetching = false;
        this._cdcPathsTbody    = null;
        this._cdcPathsToken    = (this._cdcPathsToken || 0) + 1;
        // Reset the shared-FF tracking map. Each (capture_inst →
        // [rows]) entry tells us how many paths in the current
        // load list share the same capture flop, so we can stamp
        // a "⚭ N" badge on each row when N > 1. Multi-clock-CK
        // FFs are the dominant case where N > 1: the same
        // physical flop appears once per (launch, capture) pair.
        this._cdcInstRowsMap = new Map();
        const token = this._cdcPathsToken;
        try {
            await this._app.websocketManager.readyPromise;
            const data = await this._requestWithTimeout({
                type: 'cdc_paths',
                launch_clock:  launch,
                capture_clock: capture,
                category:      category,
                pattern:       pattern,
                // Thread the active matrix mode through so the path
                // list comes from the same scenario the user clicked,
                // not whatever cmdMode happens to be on the server.
                mode:          this._cdcActiveModeName(this._cdcOverview),
                offset:        0,
                limit:         this._cdcPathsBatchSize(),
            });
            // Drop the response if a newer query has been issued.
            if (token !== this._cdcPathsToken) return;
            this._cdcPathsTotal  = (data && data.total) || 0;
            this._cdcPathsLoaded = ((data && data.paths) || []).length;
            this._renderCdcPaths(data, launch, capture);
            this._maybeTopUpCdcPaths();
        } catch (e) {
            console.warn('[CDC] paths load failed', e);
        }
    }

    // Fetch the next page and append rows to the existing table.
    // Triggered by the scroll handler on `_cdcBody` and by
    // _maybeTopUpCdcPaths when the first batch didn't fill the pane.
    async _fetchCdcPathsMore() {
        if (this._cdcPathsFetching) return;
        if (this._cdcPathsLoaded >= this._cdcPathsTotal) return;
        if (!this._cdcPathsTbody) return;
        this._cdcPathsFetching = true;
        const launch  = this._cdcCurrentLaunch;
        const capture = this._cdcCurrentCapture;
        const token = this._cdcPathsToken;
        try {
            // Same pattern-folding rule as `_loadCdcPaths` — keep the
            // bare-word substring affordance consistent across the
            // initial load and pagination fetches so the user's
            // pattern doesn't drift between the first page and the
            // scrolled-in pages.
            const rawPattern = (this._cdcCurrentPattern || '').trim();
            const pattern = rawPattern && !/[*?]/.test(rawPattern)
                ? `*${rawPattern}*`
                : rawPattern;
            const data = await this._requestWithTimeout({
                type: 'cdc_paths',
                launch_clock:  launch,
                capture_clock: capture,
                category:      this._cdcCurrentCategoryFilter || 'all',
                pattern:       pattern,
                mode:          this._cdcActiveModeName(this._cdcOverview),
                offset:        this._cdcPathsLoaded,
                limit:         this._cdcPathsBatchSize(),
            });
            if (token !== this._cdcPathsToken) return;
            const more = (data && data.paths) || [];
            this._cdcPathsLoaded += more.length;
            if (typeof data.total === 'number') {
                this._cdcPathsTotal = data.total;
            }
            this._appendCdcPathsBatch(more, launch, capture);
            this._maybeTopUpCdcPaths();
        } catch (e) {
            console.warn('[CDC] paths paginate fetch failed', e);
        } finally {
            this._cdcPathsFetching = false;
        }
    }

    // After a batch lands, fetch more if the scroll area still has
    // headroom. Same trick as the SDC tabs: a first batch that doesn't
    // fill the pane leaves no scrollbar to drag, so the scroll handler
    // never fires and the user is stuck.
    _maybeTopUpCdcPaths() {
        if (typeof requestAnimationFrame === 'undefined') return;
        requestAnimationFrame(() => {
            const el = this._cdcBody;
            if (!el) return;
            if (this._cdcPathsFetching) return;
            if (this._cdcPathsLoaded >= this._cdcPathsTotal) return;
            if (el.scrollHeight <= el.clientHeight + 16) {
                this._fetchCdcPathsMore();
            }
        });
    }

    _renderCdcPaths(data, launch, capture) {
        // Preserve the user's in-progress edits on the search input
        // across re-renders. Sequence we have to handle:
        //   1) User types `syn`, presses Enter → fetch starts,
        //      `_cdcCurrentPattern = "syn"`.
        //   2) While the fetch is in flight, user starts typing again:
        //      input now shows `sync_a` (locally edited, not yet
        //      committed via Enter).
        //   3) Fetch returns → `_renderCdcPaths` rebuilds the toolbar.
        //      Naively seeding the new input from `_cdcCurrentPattern`
        //      would clobber the user's `sync_a` with the older
        //      committed `syn`.
        // Capture the LIVE input value (and focus + caret) before the
        // rebuild so the new input restores exactly what was on screen.
        const liveInput = this._cdcSearchInput;
        const liveValue
            = liveInput ? liveInput.value : null;
        const prevSearchHadFocus
            = liveInput && document.activeElement === liveInput;
        const prevSearchSelStart = prevSearchHadFocus
            ? liveInput.selectionStart : null;
        const prevSearchSelEnd = prevSearchHadFocus
            ? liveInput.selectionEnd : null;

        // Hide the path-detail legend footer — only the path-detail
        // view populates it.
        this._hideCdcLegendFooter();

        this._cdcBody.innerHTML = '';
        const wrap = document.createElement('div');
        // No gap between the sticky header block and the table — the
        // gap was an 8px transparent strip between stickyHeader's
        // bottom and thead's top, and rows scrolled THROUGH it on the
        // way up, appearing to float above the column titles. The
        // sticky-header's own `padding-bottom + border-bottom` provides
        // the visual separation so we don't need a flex gap here.
        wrap.style.cssText = 'display:flex;flex-direction:column;';
        this._cdcBody.appendChild(wrap);

        // Breadcrumb
        // Sticky header block: groups the breadcrumb + filter bar so
        // they pin to the top of the scroll area as the user scrolls
        // through a long path list. The opaque background prevents
        // table rows from showing through. `z-index:3` keeps it above
        // the sticky thead (z-index:2) which itself sits above row
        // content (z-index default).
        const stickyHeader = document.createElement('div');
        stickyHeader.style.cssText =
            // `top:-1px` (instead of `0`) papers over a 1px sub-pixel
            // gap some browsers leave at the very top edge of a sticky
            // child of an `overflow:auto` parent — scrolled content
            // scrolls THROUGH that 1px gap. A 1px overshoot is
            // imperceptible visually but seals the seam.
            'position:sticky;top:-1px;z-index:10;'
            // Solid background — `--bg-main` is opaque (#1e1e1e dark,
            // #ffffff light). `box-shadow` gives the user an explicit
            // "this is pinned" visual, AND papers over any 1px sub-
            // pixel rendering quirk where scrolled content might leak
            // along the bottom edge.
            + 'background:var(--bg-main);'
            + 'box-shadow:0 2px 4px rgba(0,0,0,0.15);'
            // `will-change:transform` forces the browser to put this
            // element on its own compositor layer, which guarantees it
            // paints *above* the scrolling tbody regardless of any
            // stacking-context surprises further down the tree.
            + 'will-change:transform;'
            // Full width of the wrap container, irrespective of how
            // children flex inside (some children — the breadcrumb —
            // might naturally be narrower than the body table below).
            + 'width:100%;box-sizing:border-box;'
            + 'display:flex;flex-direction:column;gap:8px;'
            + 'padding-bottom:6px;'
            + 'border-bottom:1px solid var(--border-subtle);';
        wrap.appendChild(stickyHeader);

        // Sticky anchors to the scroll container's PADDING edge, not its
        // border edge. _cdcBody uses padding:8px, which means a literal
        // `top:0` (or top:-1) sticks 8px below the visual top of the
        // pane — and rows scroll through that 8px gap as they pass by.
        // Read the padding dynamically and offset by it, plus a 1px
        // overlap to seal sub-pixel seams.
        const scrollerPadTop = parseFloat(
            getComputedStyle(this._cdcBody).paddingTop) || 0;
        stickyHeader.style.top = `-${scrollerPadTop + 1}px`;

        const crumb = document.createElement('div');
        crumb.style.cssText =
            'font-size:12px;color:var(--fg-secondary);'
            + 'display:flex;align-items:baseline;gap:8px;min-width:0;';
        const back = document.createElement('a');
        back.textContent = '◂ matrix';
        back.style.cssText =
            'cursor:pointer;color:var(--accent-tab);text-decoration:underline;'
            + 'flex-shrink:0;';
        back.addEventListener('click', () => {
            if (this._cdcOverview) this._renderCdcMatrix(this._cdcOverview);
        });
        crumb.appendChild(back);
        const sep = document.createElement('span');
        sep.textContent = '·';
        sep.style.flexShrink = '0';
        sep.style.color = 'var(--fg-muted)';
        crumb.appendChild(sep);
        const lbl = document.createElement('span');
        lbl.textContent = `${launch} → ${capture}`;
        lbl.title = `${launch} → ${capture}`;
        // Clock names are short by convention but generated-clock
        // names (`my_pll/clkout_div2`) can grow — let it truncate.
        lbl.style.cssText =
            'font-weight:600;color:var(--fg-primary);min-width:0;'
            + 'overflow:hidden;text-overflow:ellipsis;white-space:nowrap;';
        crumb.appendChild(lbl);
        stickyHeader.appendChild(crumb);

        // Category filter buttons (counts from category_total).
        const cat = (data && data.category_total) || {};
        const total = (data && data.total) || 0;
        const filterBar = document.createElement('div');
        filterBar.style.cssText =
            'display:flex;gap:6px;align-items:center;font-size:12px;';
        const filterLbl = document.createElement('span');
        filterLbl.textContent = 'Filter:';
        filterLbl.style.cssText = 'color:var(--fg-muted);font-weight:600;';
        filterBar.appendChild(filterLbl);
        this._cdcCategoryBtns = this._makeFilterButtons({
            container: filterBar,
            defs: [
                { key: 'all',            label: `All (${total})` },
                { key: 'unsynchronized', label: `Unsynced (${cat.unsynced || 0})` },
                { key: 'synchronized',   label: `Synced (${cat.synced || 0})` },
                { key: 'excluded',       label: `Excluded (${cat.excluded || 0})` },
            ],
            initialKey: this._cdcCurrentCategoryFilter || 'all',
            datasetPrefix: 'cdcCat',
            // Re-fetch through _loadCdcPaths so we go down the same
            // paginated path (resets offset to 0, re-installs scroll
            // handler) instead of a one-shot full-list fetch.
            onSelect: (key) => {
                this._loadCdcPaths(launch, capture, {
                    category: key,
                    pattern: this._cdcCurrentPattern || '',
                });
            },
        });

        // Capture-pin glob/substring search — Enter-to-search (NOT
        // live), matching the Endpoints and Port Delays tabs. Live
        // filtering across a backend round-trip raced with re-renders
        // when a user typed faster than the debounce + fetch could
        // settle, occasionally clobbering in-progress input with the
        // older committed pattern; one consistent commit-on-Enter
        // rule sidesteps that whole class of issue.
        const searchSep = document.createElement('div');
        searchSep.style.cssText
            = 'width:1px;height:14px;background:var(--border);margin:0 4px;';
        filterBar.appendChild(searchSep);
        const cdcSearch = document.createElement('input');
        cdcSearch.type = 'text';
        cdcSearch.placeholder
            = 'Filter capture pins (e.g. ff_*/D or sync_a) — Enter to search';
        cdcSearch.title
            = 'Glob/substring filter on capture-pin path. Wildcards: '
            + '`*` matches any sequence, `?` matches one character. '
            + 'A pattern with no wildcards matches as a substring '
            + '(e.g. `sync` matches `top/sync_b/D`). '
            + 'Press Enter or click Search to apply; clearing the '
            + 'field and pressing Enter restores all paths in this '
            + 'category.';
        cdcSearch.style.cssText
            = 'flex:1;min-width:120px;padding:2px 6px;font-size:12px;'
            + 'font-family:monospace;background:var(--bg-input);'
            + 'color:var(--fg-primary);border:1px solid var(--border);'
            + 'border-radius:3px;outline:none;';
        // Prefer the live (uncommitted) value if there was one — see
        // the rationale on the capture at the top of this method.
        // Falls back to the committed pattern on a fresh render where
        // no prior input element existed.
        cdcSearch.value = liveValue != null
            ? liveValue
            : (this._cdcCurrentPattern || '');
        this._cdcSearchInput = cdcSearch;
        const triggerSearch = () => {
            const raw = cdcSearch.value.trim();
            this._cdcCurrentPattern = raw;
            this._loadCdcPaths(launch, capture, {
                category: this._cdcCurrentCategoryFilter || 'all',
                pattern: raw,
            });
        };
        cdcSearch.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') triggerSearch();
        });
        filterBar.appendChild(cdcSearch);
        const cdcSearchBtn = document.createElement('button');
        cdcSearchBtn.textContent = 'Search';
        cdcSearchBtn.style.cssText
            = 'padding:2px 8px;font-size:12px;cursor:pointer;'
            + 'background:var(--bg-input);color:var(--fg-primary);'
            + 'border:1px solid var(--border);border-radius:3px;';
        cdcSearchBtn.addEventListener('click', triggerSearch);
        filterBar.appendChild(cdcSearchBtn);

        stickyHeader.appendChild(filterBar);

        // Two-table layout for rock-solid sticky column headers.
        // Single-table sticky-<th> works in theory but requires
        // measuring the breadcrumb height after layout (rAF) and is
        // fragile when the breadcrumb wraps or the user scrolls
        // before the first paint settles. Splitting the column
        // headers into their own table that lives INSIDE the sticky
        // block means the headers can never be detached from the
        // breadcrumb, can never have a transparent gap above them,
        // and need no measurement. Both tables share `<colgroup>`
        // widths so columns line up; `table-layout:fixed` makes the
        // pin column flex to fill the rest in both.
        //
        // `border-collapse:separate` chosen for both because
        // `collapse` leaks 1px of row content through the shared
        // collapsed border as the body scrolls past sticky headers
        // in every browser. Row separators live on each `<td>`
        // bottom-border (set in _buildCdcPathRow).
        const COL_WIDTHS = [null, '140px', '120px', '160px'];
        const COL_HEADERS =
            ['Capture pin', 'Capture cell', 'Category', 'Sync chain'];
        const makeColgroup = () => {
            const cg = document.createElement('colgroup');
            for (const w of COL_WIDTHS) {
                const col = document.createElement('col');
                if (w) col.style.width = w;
                cg.appendChild(col);
            }
            return cg;
        };
        const tableStyle =
            'border-collapse:separate;border-spacing:0;'
            + 'font-size:12px;font-family:monospace;'
            + 'width:100%;table-layout:fixed;';

        // ── Header table: lives INSIDE the sticky block ────────
        const headerTbl = document.createElement('table');
        headerTbl.style.cssText = tableStyle;
        headerTbl.appendChild(makeColgroup());
        const thead = document.createElement('thead');
        const hr = document.createElement('tr');
        for (const h of COL_HEADERS) {
            const th = document.createElement('th');
            th.style.cssText =
                'padding:4px 8px;border-bottom:1px solid var(--border);' +
                'background:var(--bg-header);text-align:left;';
            th.textContent = h;
            hr.appendChild(th);
        }
        thead.appendChild(hr);
        headerTbl.appendChild(thead);
        stickyHeader.appendChild(headerTbl);
        // The column-header row is inside stickyHeader now, so the
        // breadcrumb's bottom-border separator is redundant — remove
        // it; the thead's own bottom-border serves as the divider.
        stickyHeader.style.borderBottom = 'none';
        stickyHeader.style.paddingBottom = '0';

        // ── Body table: just the rows ─────────────────────────
        const tbl = document.createElement('table');
        tbl.style.cssText = tableStyle;
        tbl.appendChild(makeColgroup());
        const tbody = document.createElement('tbody');
        const paths = (data && data.paths) || [];
        for (const p of paths) {
            tbody.appendChild(this._buildCdcPathRow(p, launch, capture));
        }
        tbl.appendChild(tbody);
        wrap.appendChild(tbl);

        // Footer / pagination status — counts visible vs total so the
        // user knows whether more is coming as they scroll.
        const footer = document.createElement('div');
        footer.style.cssText =
            'padding:8px;text-align:center;font-size:12px;'
            + 'color:var(--fg-muted);font-style:italic;';
        wrap.appendChild(footer);
        this._cdcPathsFooter = footer;
        this._cdcPathsTbody = tbody;
        this._refreshCdcPathsFooter();

        if (paths.length === 0 && this._cdcPathsTotal === 0) {
            const msg = document.createElement('div');
            msg.style.cssText =
                'padding:16px;color:var(--fg-muted);font-style:italic;';
            msg.textContent = 'No paths matching the current filter.';
            wrap.appendChild(msg);
        }

        // Install the infinite-scroll handler on the scrollable body
        // (the CDC tab uses `_cdcBody` as its scroll container).
        this._installCdcPathsInfiniteScroll();

        // Restore focus + caret on the search input — see the capture
        // at the top of this method for the rationale.
        if (prevSearchHadFocus && this._cdcSearchInput) {
            this._cdcSearchInput.focus();
            try {
                this._cdcSearchInput.setSelectionRange(
                    prevSearchSelStart, prevSearchSelEnd);
            } catch (_) {
                // Some input types throw on setSelectionRange; the
                // focus call above already gave the user a usable
                // editing context.
            }
        }
    }

    // Build one row of the path table. Extracted so _renderCdcPaths
    // (initial load) and _appendCdcPathsBatch (paginated drip) share
    // the same DOM shape.
    _buildCdcPathRow(p, launch, capture) {
        const tr = document.createElement('tr');
        // With border-collapse:separate, <tr> borders don't render —
        // each <td> carries its own bottom border so the row separator
        // still appears between rows. Centralised in tdBase so all
        // four cells share the same trailing edge without drift.
        const tdBase =
            'padding:3px 8px;border-bottom:1px solid var(--border-subtle);';
        // Capture pin — clickable link that opens the per-path
        // detail view directly. Earlier this column linkified the
        // pin to the inspector and a separate "detail ▸" column
        // launched the path-detail view; user feedback was that
        // having two affordances on one row was confusing (you'd
        // click the pin expecting timing detail and instead get an
        // inspector selection). Inspector-style selection is still
        // reachable through other parts of the UI; the row itself
        // now has ONE click action and the trailing column is gone.
        // `display:block` on the inner span activates ellipsis
        // inside a fixed-layout table cell.
        const tdPin = document.createElement('td');
        tdPin.style.cssText = tdBase + 'overflow:hidden;';
        // Flex wrapper so the (potentially-long) pinSpan can
        // ellipsize while the trailing "⚭ N" shared-FF badge
        // stays anchored to the right of the cell.
        const pinWrap = document.createElement('div');
        pinWrap.style.cssText
            = 'display:flex;align-items:center;gap:6px;min-width:0;';
        const pinSpan = document.createElement('span');
        pinSpan.style.cssText
            = 'flex:1;min-width:0;' + TRUNCATE_PATH_CSS
            + 'cursor:pointer;color:var(--accent-tab);'
            + 'text-decoration:underline;';
        pinSpan.textContent = p.capture_pin;
        pinSpan.title = `${p.capture_pin} — click for the path detail`;
        pinSpan.addEventListener('click',
            () => this._loadCdcPathDetail(p, launch, capture));
        pinWrap.appendChild(pinSpan);
        // Shared-FF badge — hidden by default, shown when this
        // row's `capture_inst` appears more than once in the
        // currently-loaded list. The same FF can legitimately
        // contribute multiple (launch, capture) pairs (e.g. a
        // multi-clock-CK flop), and the badge tells the user
        // "you'll see N rows for this same FF — fixing it once
        // addresses all of them."
        const sharesBadge = document.createElement('span');
        sharesBadge.style.cssText
            = 'flex-shrink:0;font-size:11px;'
            + 'color:var(--fg-muted);font-style:italic;'
            + 'display:none;';
        pinWrap.appendChild(sharesBadge);
        tdPin.appendChild(pinWrap);
        tr.appendChild(tdPin);

        // Register this row for shared-FF tracking. Refreshing
        // every existing row in the same group keeps the badge
        // count up to date as paginated batches stream in.
        if (p.capture_inst) {
            if (!this._cdcInstRowsMap) {
                this._cdcInstRowsMap = new Map();
            }
            if (!this._cdcInstRowsMap.has(p.capture_inst)) {
                this._cdcInstRowsMap.set(p.capture_inst, []);
            }
            const group = this._cdcInstRowsMap.get(p.capture_inst);
            group.push(sharesBadge);
            this._refreshCdcSharesBadges(group, p.capture_inst);
        }
        // Capture cell (master). Default end-truncation is fine here.
        const tdCell = document.createElement('td');
        tdCell.style.cssText =
            tdBase + 'color:var(--fg-secondary);'
            + 'overflow:hidden;text-overflow:ellipsis;white-space:nowrap;';
        tdCell.textContent = p.capture_cell || '—';
        tdCell.title = p.capture_cell || '';
        tr.appendChild(tdCell);
        // Category badge with traffic-light fill.
        const tdCat = document.createElement('td');
        tdCat.style.cssText = tdBase;
        const catEl = document.createElement('span');
        catEl.textContent = p.category;
        catEl.style.cssText =
            'padding:1px 6px;border-radius:3px;font-weight:600;';
        if (p.category === 'unsynchronized') {
            catEl.style.background = 'rgba(220, 64, 64, 0.30)';
            catEl.title =
                'no synchronizer detected on the capture side — ' +
                'this is the actionable bug bucket';
        } else if (p.category === 'synchronized') {
            catEl.style.background = 'rgba(76, 175, 80, 0.25)';
            catEl.title =
                'capture flop is followed by a synchronizer ' +
                'matching one of the detector tiers';
        } else {
            catEl.style.background = 'rgba(150, 150, 150, 0.20)';
            catEl.style.color = 'var(--fg-muted)';
            catEl.title =
                'covered by an SDC clock_groups / false_path entry — ' +
                'STA does not analyse this path';
        }
        tdCat.appendChild(catEl);
        tr.appendChild(tdCat);
        // Sync chain.
        const tdSync = document.createElement('td');
        tdSync.style.cssText = tdBase;
        tdSync.appendChild(this._renderCdcSyncChainCell(p));
        tr.appendChild(tdSync);
        return tr;
    }

    _appendCdcPathsBatch(paths, launch, capture) {
        if (!this._cdcPathsTbody) return;
        for (const p of paths) {
            this._cdcPathsTbody.appendChild(
                this._buildCdcPathRow(p, launch, capture));
        }
        this._refreshCdcPathsFooter();
    }

    _refreshCdcPathsFooter() {
        const f = this._cdcPathsFooter;
        if (!f) return;
        const loaded = this._cdcPathsLoaded || 0;
        const total  = this._cdcPathsTotal  || 0;
        if (loaded >= total) {
            f.textContent = total > 0
                ? `${total} path${total === 1 ? '' : 's'}`
                : '';
        } else {
            f.textContent = `Showing ${loaded} of ${total} — scroll to load more…`;
        }
    }

    _installCdcPathsInfiniteScroll() {
        if (this._cdcPathsScrollHandler) return;  // attached once
        const handler = () => {
            const el = this._cdcBody;
            if (!el) return;
            if (this._cdcPathsFetching) return;
            if (!this._cdcPathsTbody) return;
            if (this._cdcPathsLoaded >= this._cdcPathsTotal) return;
            const slack = el.clientHeight * 1.5;
            if (el.scrollTop + el.clientHeight + slack >= el.scrollHeight) {
                this._fetchCdcPathsMore();
            }
        };
        this._cdcPathsScrollHandler = handler;
        if (this._cdcBody) this._cdcBody.addEventListener('scroll', handler);
    }

    // Render the "Sync chain" cell — short text + a tooltip explaining
    // exactly how the classifier reached this conclusion. Surfacing the
    // *why* is the whole reason this column exists; the user shouldn't
    // have to guess what changed when a row goes from unsynced to synced.
    _renderCdcSyncChainCell(p) {
        const span = document.createElement('span');
        // The cell sits in a fixed-layout table column, so anything
        // long (a whitelist pattern, a deep-depth label) would push
        // outside its column without these — same trick as the pin
        // column elsewhere in the table.
        span.style.cssText =
            'color:var(--fg-secondary);display:block;'
            + 'overflow:hidden;text-overflow:ellipsis;white-space:nowrap;';
        const kind = p.sync_chain_kind || 'none';
        const depth = p.sync_chain_depth || 0;
        if (kind === 'ff_chain') {
            span.textContent = `${depth}FF chain`;
            span.title =
                `Detected via FF→FF heuristic: the capture flop's Q feeds ` +
                `into ${depth - 1} additional flop(s) in the same capture ` +
                `domain with no combinational logic in between (buffers and ` +
                `inverters are tolerated and don't reset the chain count).`;
        } else if (kind === 'liberty_sync') {
            span.textContent = `liberty sync (depth ${depth})`;
            span.title =
                `Capture cell uses a Liberty 'statetable' sequential body, ` +
                `which typically means a vendor-provided synchronizer cell. ` +
                `Detected via LibertyCell::statetable() != nullptr. Depth ` +
                `is the count of internal storage nodes in the statetable.`;
        } else if (kind === 'composite') {
            span.textContent = `composite chain (depth ${depth})`;
            span.title =
                `Mixed chain: at least one ff-group flop AND at least one ` +
                `Liberty 'statetable' synchronizer cell in series in the ` +
                `same capture domain. Reported depth sums each ff flop as ` +
                `1 and each statetable cell as its internal-node count, ` +
                `so a SYNC2_X1 followed by a regular DFF reports depth 3. ` +
                `Inspect each stage in the path-detail diagram to audit ` +
                `the chain composition.`;
        } else if (kind === 'whitelisted') {
            const which = p.whitelist_match || '?';
            const pat = p.whitelist_pattern || '';
            span.textContent = `whitelist (${which})`;
            span.title =
                `User whitelist match: capture flop's ${which} matched ` +
                `pattern '${pat}' in CDC settings. Edit the list via the ` +
                `"⚙ CDC settings" button on the toolbar.`;
        } else {
            span.textContent = '— none';
            span.style.color = 'var(--fg-muted)';
            span.title =
                'No synchronizer detected. The capture flop is the only ' +
                'register before downstream combinational logic, and no ' +
                'whitelist entry matched. Add a sync chain in RTL, or ' +
                'whitelist the cell/instance via "⚙ CDC settings".';
        }
        return span;
    }

    async _loadCdcPathDetail(pathRow, launch, capture) {
        try {
            await this._app.websocketManager.readyPromise;
            const data = await this._requestWithTimeout({
                type: 'cdc_path_detail',
                capture_odb_type: pathRow.odb_type,
                capture_odb_id:   pathRow.odb_id,
                // Threading the matrix's launch/capture clocks into
                // the request so the backend's stage emit matches
                // the row the user actually clicked. Without this,
                // multi-launch endpoints could emit a different
                // launch clock than the path-list row showed.
                launch_clock:  launch,
                capture_clock: capture,
                mode: this._cdcActiveModeName(this._cdcOverview),
            });
            this._renderCdcPathDetail(data, pathRow, launch, capture);
        } catch (e) {
            console.warn('[CDC] path detail load failed', e);
        }
    }

    _renderCdcPathDetail(data, pathRow, launch, capture) {
        this._cdcBody.innerHTML = '';
        // Same path-list cleanup as _renderCdcMatrix: bumping the
        // token cancels any in-flight pagination, and clearing the
        // tbody/footer refs makes the append callback a no-op.
        this._cdcPathsTbody  = null;
        this._cdcPathsFooter = null;
        this._cdcPathsToken  = (this._cdcPathsToken || 0) + 1;
        const wrap = document.createElement('div');
        wrap.style.cssText = 'display:flex;flex-direction:column;gap:10px;';
        this._cdcBody.appendChild(wrap);

        // Breadcrumb back to the path list. Flex layout so the
        // instance segment can take the leftover width and truncate
        // when the panel is narrow — the matrix and clock-pair links
        // stay readable as fixed labels at the start.
        const crumb = document.createElement('div');
        crumb.style.cssText =
            'font-size:12px;color:var(--fg-secondary);'
            + 'display:flex;align-items:baseline;gap:8px;min-width:0;';
        const backMatrix = document.createElement('a');
        backMatrix.textContent = '◂ matrix';
        backMatrix.style.cssText =
            'cursor:pointer;color:var(--accent-tab);text-decoration:underline;'
            + 'flex-shrink:0;';
        backMatrix.addEventListener('click', () => {
            if (this._cdcOverview) this._renderCdcMatrix(this._cdcOverview);
        });
        crumb.appendChild(backMatrix);
        const sep1 = document.createElement('span');
        sep1.textContent = '·';
        sep1.style.flexShrink = '0';
        crumb.appendChild(sep1);
        const backList = document.createElement('a');
        backList.textContent = `${launch} → ${capture}`;
        backList.style.cssText =
            'cursor:pointer;color:var(--accent-tab);text-decoration:underline;'
            + 'flex-shrink:0;';
        backList.addEventListener('click', () =>
            this._loadCdcPaths(launch, capture));
        crumb.appendChild(backList);
        const sep2 = document.createElement('span');
        sep2.textContent = '·';
        sep2.style.flexShrink = '0';
        sep2.style.color = 'var(--fg-muted)';
        crumb.appendChild(sep2);
        // Instance segment — the only path-shaped element on the row.
        const instSeg = document.createElement('span');
        instSeg.textContent = pathRow.capture_inst;
        instSeg.title = pathRow.capture_inst;
        instSeg.style.cssText =
            'color:var(--fg-primary);font-weight:600;flex:1;min-width:0;'
            + TRUNCATE_PATH_CSS;
        crumb.appendChild(instSeg);
        wrap.appendChild(crumb);

        // Summary line — explains the classification in plain English.
        const sc = (data && data.sync_chain) || { kind: 'none', depth: 0 };
        const summary = document.createElement('div');
        summary.style.cssText =
            'padding:6px 10px;background:var(--bg-input);border-radius:3px;' +
            'font-size:12px;line-height:1.5;';
        const stages = (data && data.stages) || [];
        // Stage-shape inventory drives the summary suffix so the user
        // sees what was walked: launch-side comb gates, the crossover,
        // sync stages.
        const launchStages = stages.filter(s =>
            s && s.is_launch && (s.kind === 'register' || s.kind === 'port'));
        const combStages = stages.filter(s => s && s.kind === 'comb');
        const launchSide = (launchStages.length > 0 || combStages.length > 0)
            ? `${launchStages.length} launch · ${combStages.length} comb · `
            : '';
        let summaryText;
        if (sc.kind === 'ff_chain') {
            summaryText =
                `${launch} → ${capture} · ${launchSide}` +
                `${sc.depth}FF synchronizer detected (FF→FF chain)`;
        } else if (sc.kind === 'liberty_sync') {
            summaryText =
                `${launch} → ${capture} · ${launchSide}` +
                `Liberty statetable sync cell (depth ${sc.depth})`;
        } else if (sc.kind === 'composite') {
            summaryText =
                `${launch} → ${capture} · ${launchSide}` +
                `composite synchronizer detected (FF + statetable, ` +
                `depth ${sc.depth})`;
        } else if (sc.kind === 'whitelisted') {
            summaryText =
                `${launch} → ${capture} · ${launchSide}` +
                `whitelisted (${sc.whitelist_match || '?'} = ` +
                `${sc.whitelist_pattern || ''})`;
        } else {
            summaryText =
                `${launch} → ${capture} · ${launchSide}` +
                'no sync chain detected';
        }
        summary.textContent = summaryText;
        wrap.appendChild(summary);

        // Stage diagram — top-to-bottom flow. Each card surfaces:
        //   • role badge (CROSSOVER / SYNC #N)
        //   • instance path + master cell  (linkified to instance)
        //   • D pin and Q pin paths        (linkified to each pin)
        //   • clock domain on each pin so the colour-flip at the
        //     crossover (D=launch, Q=capture) is visible without
        //     having to compare cards side-by-side
        //
        // Vertical layout chosen over the original left-to-right flow
        // so cards never wrap onto a second row on narrow widget
        // panes — each card gets its full width and the down-arrow
        // chain reads as a single linear sequence regardless of
        // viewport size. Width follows the parent so the diagram is
        // responsive: docked-panel narrow → cards squish + paths
        // truncate; full-window wide → cards expand to show more
        // of each path.
        const diagram = document.createElement('div');
        diagram.style.cssText =
            'display:flex;flex-direction:column;align-items:stretch;' +
            'gap:4px;padding:8px 4px;width:100%;min-width:0;';
        // Track stage indices for sync-stage banner numbering — sync
        // stages are everything AFTER the crossover in the array, and
        // we want to label them "sync stage 1" / "sync stage 2" rather
        // than using the raw array index (which now includes launch +
        // comb + capture before the syncs).
        let captureIdx = -1;
        for (let i = 0; i < stages.length; ++i) {
            if (stages[i] && stages[i].is_capture) { captureIdx = i; break; }
        }
        // True iff at least one downstream sync stage exists. Used by
        // the capture-flop banner: "crossover · stage 1" only makes
        // sense when there's a stage 2 to follow. Single-cell synced
        // chains (e.g. one SYNC2 alone, or a whitelisted single flop)
        // get a plain "crossover" banner instead — still green to
        // mark the chain as synced.
        const hasDownstreamSync = stages.some(st => st && st.is_sync_stage);
        for (let i = 0; i < stages.length; ++i) {
            const s = stages[i];
            if (i > 0) {
                // Inter-stage region: net row plus an expander when the
                // upstream stage carried any silently-collapsed buf/inv
                // pass-throughs. Default-collapsed so the diagram stays
                // tight; clicking the summary reveals the buffer chain.
                const prevStage = stages[i - 1];
                const prevNet = prevStage && prevStage.out_net;
                const passthroughs = (prevStage
                    && Array.isArray(prevStage.passthroughs_after))
                    ? prevStage.passthroughs_after : [];
                diagram.appendChild(
                    this._renderCdcInterStageGap(prevNet, passthroughs));
            }
            const syncOrdinal = (captureIdx >= 0 && i > captureIdx)
                ? (i - captureIdx) : 0;
            diagram.appendChild(
                this._renderCdcStageCard(s, syncOrdinal, launch, capture, sc,
                    { hasDownstream: hasDownstreamSync }));
        }
        if (stages.length === 0) {
            const msg = document.createElement('div');
            msg.style.cssText =
                'padding:8px;color:var(--fg-muted);font-style:italic;';
            msg.textContent = 'No stage data available.';
            diagram.appendChild(msg);
        }
        wrap.appendChild(diagram);

        // Sticky-bottom legend — populated AFTER stages are
        // rendered so it can enumerate exactly the clocks visible
        // on this diagram.
        this._showCdcLegendFooter(stages, launch, capture);
    }

    // Inter-stage connector between two stage cards. Shows the net
    // leaving the upstream stage as a clickable centred row; when the
    // back-walk silently collapsed buf/inv pass-throughs in the gap,
    // wraps the row in a `<details>` element so the user can expand
    // and see each collapsed cell. Click-through on the net dispatches
    // `inspect` with `odb_type:"net"` (resolveByOdb in request_handler
    // resolves to a dbNet and hands off through the inspect pipeline).
    _renderCdcInterStageGap(net, passthroughs) {
        const hasNet = net && net.odb_type && net.odb_id != null;
        const passes = Array.isArray(passthroughs) ? passthroughs : [];

        // Build the centred net row used in both collapsed and
        // expanded states. Reused for the intermediate nets between
        // pass-throughs when the user expands.
        const netRow = (n) => {
            const row = document.createElement('div');
            row.style.cssText =
                'display:flex;align-items:center;gap:6px;' +
                'padding:2px 4px;font-family:monospace;font-size:12px;' +
                'color:var(--fg-muted);';
            const ruleL = document.createElement('span');
            ruleL.textContent = '──';
            ruleL.style.flexShrink = '0';
            const label = document.createElement('span');
            label.textContent = (n && n.name) || '(net)';
            label.title = (n && n.name) || '';
            label.style.cssText =
                'flex:1;min-width:0;text-align:center;' + TRUNCATE_PATH_CSS;
            if (n && n.odb_type && n.odb_id != null) {
                this._linkifyPin(label, n);
            }
            const ruleR = document.createElement('span');
            ruleR.textContent = '──';
            ruleR.style.flexShrink = '0';
            row.appendChild(ruleL);
            row.appendChild(label);
            row.appendChild(ruleR);
            return row;
        };

        // No net AND no passthroughs — keep the legacy arrow indicator.
        if (!hasNet && passes.length === 0) {
            const arrow = document.createElement('div');
            arrow.textContent = '↓';
            arrow.style.cssText =
                'color:var(--fg-muted);font-weight:bold;' +
                'font-size:16px;text-align:center;line-height:1;' +
                'padding:2px 0;';
            return arrow;
        }

        // No passthroughs — just the net row, no expander needed.
        if (passes.length === 0) {
            return netRow(net);
        }

        // Pass-throughs present — wrap the net row in a <details> so
        // the user can expand to see each buf/inv mini-card and the
        // chain of intermediate nets. <details> handles state natively
        // so we don't need to wire toggle handlers manually.
        const details = document.createElement('details');
        details.style.cssText = 'margin:0;';
        const summary = document.createElement('summary');
        summary.style.cssText =
            'list-style:none;cursor:pointer;outline:none;' +
            'display:flex;align-items:center;gap:6px;' +
            'padding:2px 4px;font-family:monospace;font-size:12px;' +
            'color:var(--fg-muted);';
        // Hide the default disclosure triangle — we provide our own.
        const styleEl = document.createElement('style');
        styleEl.textContent =
            'details > summary::-webkit-details-marker { display:none; }';
        details.appendChild(styleEl);
        const summaryRuleL = document.createElement('span');
        summaryRuleL.textContent = '──';
        summaryRuleL.style.flexShrink = '0';
        const summaryLabel = document.createElement('span');
        summaryLabel.style.cssText =
            'flex:1;min-width:0;text-align:center;' + TRUNCATE_PATH_CSS;
        const cellHint = passes.length === 1
            ? `+ 1 hidden cell`
            : `+ ${passes.length} hidden cells`;
        summaryLabel.textContent = hasNet
            ? `${net.name}  ▸  ${cellHint}`
            : cellHint;
        summaryLabel.title = passes
            .map(p => `${p.cell || '?'}  ${p.instance || '?'}`)
            .join('\n');
        const summaryRuleR = document.createElement('span');
        summaryRuleR.textContent = '──';
        summaryRuleR.style.flexShrink = '0';
        summary.appendChild(summaryRuleL);
        summary.appendChild(summaryLabel);
        summary.appendChild(summaryRuleR);
        details.appendChild(summary);

        // Expanded body: leading net (if any) → mini-card → net …
        // → mini-card → trailing net (the last passthrough's out_net).
        const body = document.createElement('div');
        body.style.cssText =
            'display:flex;flex-direction:column;align-items:stretch;' +
            'gap:2px;padding:4px 0 2px;';
        if (hasNet) body.appendChild(netRow(net));
        for (let i = 0; i < passes.length; ++i) {
            body.appendChild(this._renderCdcPassthroughCard(passes[i]));
            // Net AFTER this pass-through. The last one's out_net feeds
            // into the next stage's input.
            if (passes[i].out_net) body.appendChild(netRow(passes[i].out_net));
        }
        details.appendChild(body);
        return details;
    }

    // Mini-card for a collapsed buf/inv pass-through. Compact layout —
    // banner + instance row only — so a chain of pass-throughs doesn't
    // dominate the diagram when expanded. Linkified to instance and
    // both pins via the existing inspect plumbing.
    _renderCdcPassthroughCard(pt) {
        const card = document.createElement('div');
        card.style.cssText =
            'border:1px dashed var(--border);border-radius:3px;' +
            'background:var(--bg-input);font-family:monospace;font-size:11px;' +
            'min-width:0;overflow:hidden;opacity:0.85;';
        const banner = document.createElement('div');
        banner.style.cssText =
            'padding:2px 6px;font-size:11px;letter-spacing:0.5px;' +
            'text-transform:uppercase;font-weight:600;' +
            'background:var(--bg-header);color:var(--fg-muted);';
        banner.textContent = `pass · ${pt.cell || 'gate'}`;
        banner.title =
            'Pass-through cell (buffer or inverter) collapsed by the ' +
            'launch-side back-walk. Doesn\'t change the data — only ' +
            'restores or inverts the polarity. Click to inspect.';
        card.appendChild(banner);

        const row = document.createElement('div');
        row.style.cssText =
            'padding:3px 6px;display:flex;align-items:baseline;' +
            'gap:6px;min-width:0;';
        const inst = document.createElement('span');
        inst.style.cssText =
            'flex:1;min-width:0;color:var(--fg-secondary);' + TRUNCATE_PATH_CSS;
        inst.textContent = pt.instance || '?';
        inst.title = pt.instance || '';
        this._linkifyPin(inst, pt);
        row.appendChild(inst);
        card.appendChild(row);
        return card;
    }

    // Deterministic soft tint for a clock domain. Same clock name → same
    // colour every time so the user can match cards across diagrams.
    // Returns null if no clock is supplied.
    _cdcClockTint(clockName, alpha) {
        if (!clockName) return null;
        let h = 0;
        for (let i = 0; i < clockName.length; ++i) {
            h = ((h << 5) - h + clockName.charCodeAt(i)) | 0;
        }
        const hue = ((h % 360) + 360) % 360;
        const a = alpha == null ? 0.10 : alpha;
        return `hsla(${hue}, 60%, 55%, ${a})`;
    }

    // Walk every stage in a path-detail response and collect the
    // unique clock names that appear anywhere — main `s.clock`,
    // explicit `launch_clock` / `capture_clock`, per-pin
    // `in_pin_clocks`, and aux-input `clocks` arrays. Used by the
    // path-detail legend footer so every tint shown on the diagram
    // is named in the key.
    _cdcStagesClocks(stages) {
        const set = new Set();
        const add = (c) => { if (c) set.add(c); };
        for (const s of stages || []) {
            if (!s) continue;
            add(s.clock);
            add(s.launch_clock);
            add(s.capture_clock);
            for (const c of (s.in_pin_clocks || [])) add(c);
            for (const aux of (s.aux_in_pins || [])) {
                for (const c of (aux.clocks || [])) add(c);
            }
        }
        // Stable order — alphabetical by clock name keeps the
        // legend in a predictable layout across re-renders.
        return Array.from(set).sort();
    }

    // Populate the sticky-bottom legend footer with everything
    // visible on the current path detail: a clock-domain swatch
    // per clock seen, and a badge key for the role banners on the
    // stage cards. Called from `_renderCdcPathDetail`; the matrix
    // and path-list views call `_hideCdcLegendFooter` instead.
    _showCdcLegendFooter(stages, launch, capture) {
        const footer = this._cdcLegendFooter;
        if (!footer) return;
        footer.innerHTML = '';
        footer.style.display = 'flex';

        // ── Clock-domain swatches ──────────────────────────────
        const clocks = this._cdcStagesClocks(stages);
        const clkRow = document.createElement('div');
        clkRow.style.cssText
            = 'display:flex;align-items:center;gap:10px;flex-wrap:wrap;';
        const clkLead = document.createElement('span');
        clkLead.textContent = 'clock domains:';
        clkLead.style.cssText
            = 'font-style:italic;color:var(--fg-muted);';
        clkRow.appendChild(clkLead);
        for (const c of clocks) {
            const item = document.createElement('span');
            item.style.cssText
                = 'display:inline-flex;align-items:center;gap:4px;'
                + 'font-family:monospace;color:var(--fg-primary);';
            const swatch = document.createElement('span');
            swatch.style.cssText
                = 'display:inline-block;width:14px;height:10px;'
                + 'border:1px solid var(--border);border-radius:2px;'
                + `background:${this._cdcClockTint(c, 0.35)};`;
            item.appendChild(swatch);
            const label = document.createElement('span');
            label.textContent = c;
            // Mark launch / capture clocks so the user can tell
            // them apart at a glance — short suffix tags.
            if (c === launch && c === capture) {
                label.textContent += ' (launch + capture)';
            } else if (c === launch) {
                label.textContent += ' (launch)';
            } else if (c === capture) {
                label.textContent += ' (capture)';
            }
            item.appendChild(label);
            item.title = `Stage cards in clock domain "${c}" carry `
                + 'this tint on their body.';
            clkRow.appendChild(item);
        }
        if (clocks.length === 0) {
            const empty = document.createElement('span');
            empty.textContent = '(no clock domains on this path)';
            empty.style.cssText = 'color:var(--fg-muted);';
            clkRow.appendChild(empty);
        }
        footer.appendChild(clkRow);

        // ── Banner / badge key ────────────────────────────────
        const badgeRow = document.createElement('div');
        badgeRow.style.cssText
            = 'display:flex;align-items:center;gap:10px;flex-wrap:wrap;';
        const badgeLead = document.createElement('span');
        badgeLead.textContent = 'banners:';
        badgeLead.style.cssText
            = 'font-style:italic;color:var(--fg-muted);';
        badgeRow.appendChild(badgeLead);
        const badgeItem = (text, bg, tooltip) => {
            const span = document.createElement('span');
            span.style.cssText
                = 'display:inline-flex;align-items:center;'
                + 'padding:1px 6px;border-radius:3px;'
                + 'font-size:10px;letter-spacing:0.6px;'
                + 'text-transform:uppercase;font-weight:600;'
                + `background:${bg};color:var(--fg-primary);`;
            span.textContent = text;
            span.title = tooltip;
            return span;
        };
        badgeRow.appendChild(badgeItem(
            'launch', 'rgba(80, 140, 220, 0.20)',
            'The launch flop / port — start of the path. Its CK '
            + 'sets the launch domain; its Q drives the path.'));
        badgeRow.appendChild(badgeItem(
            'comb · X', 'rgba(150, 150, 150, 0.20)',
            'Combinational gate on the launch-side back-walk. '
            + '"X" is the cell master. Same-domain inputs are '
            + 'expected here; multi-domain inputs upgrade the '
            + 'banner to "⚠ domain mix".'));
        badgeRow.appendChild(badgeItem(
            '⚠ domain mix · X', 'rgba(220, 64, 64, 0.20)',
            'Combinational gate whose inputs span more than one '
            + 'clock domain — the CDC bug originates here, NOT '
            + 'at the downstream capture flop.'));
        badgeRow.appendChild(badgeItem(
            'crossover · stage 1', 'rgba(76, 175, 80, 0.18)',
            'Capture flop — D is in the launch domain, CK is in '
            + 'the capture domain. A sync chain was detected '
            + 'downstream so this flop is also the meta-resolver '
            + '(stage 1) of that chain.'));
        badgeRow.appendChild(badgeItem(
            '⚠ crossover', 'rgba(220, 64, 64, 0.20)',
            'Capture flop with NO synchronizer detected. The '
            + 'metastability risk is live at this exact pin.'));
        badgeRow.appendChild(badgeItem(
            'sync stage N', 'rgba(76, 175, 80, 0.18)',
            'Subsequent sync flops downstream of the crossover '
            + '— each adds margin against metastability '
            + 'propagation.'));
        footer.appendChild(badgeRow);
    }

    _hideCdcLegendFooter() {
        if (this._cdcLegendFooter) {
            this._cdcLegendFooter.style.display = 'none';
            this._cdcLegendFooter.innerHTML = '';
        }
    }

    // ── Per-pin fan-in expansion ────────────────────────────────
    //
    // Click on a clock chip → trace fan-in for that clock back to
    // its first sequential ancestor and render the resulting
    // ancestor stages ABOVE the originating stage card. Re-clicking
    // the same chip collapses the expansion back. Result is cached
    // in `_cdcFanInCache` for the session so collapse-then-expand
    // doesn't re-walk the graph.
    //
    // Cache key: `${pin_odb_type}|${pin_odb_id}|${clock}`. Cache
    // value: the parsed `stages[]` array from the backend response.
    // Cleared on tab refresh / new scan.

    _cdcFanInCacheKey(pinRef, clock) {
        if (!pinRef || pinRef.odb_id == null) return null;
        return `${pinRef.odb_type || ''}|${pinRef.odb_id}|${clock || ''}`;
    }

    // Walk a group of shared-FF badges and apply the current
    // count to each. When count is 1, hide the badge; when > 1,
    // show "⚭ N" plus a tooltip explaining the convergence.
    // Called from `_buildCdcPathRow` every time a row joins the
    // group so existing badges stay in sync as paginated batches
    // arrive.
    _refreshCdcSharesBadges(group, instName) {
        if (!Array.isArray(group)) return;
        const n = group.length;
        for (const badge of group) {
            if (!badge) continue;
            if (n > 1) {
                badge.textContent = `⚭ ${n}`;
                badge.title
                    = `${instName} appears in ${n} paths in this `
                    + 'list — a single physical flop typically with '
                    + 'a multi-clock CK that contributes one '
                    + '(launch, capture) pair per clock combination. '
                    + 'Fixing the synchronization at this flop once '
                    + 'addresses all of them.';
                badge.style.display = '';
            } else {
                badge.textContent = '';
                badge.style.display = 'none';
            }
        }
    }

    // Append a "↑ also reached via clk_X (from <pin>)" line to a
    // stage card that another expansion just converged onto. The
    // first call creates the surrounding annotation block (with
    // its own header); subsequent calls add lines into the same
    // block. The line element is stashed back on the originating
    // chip so a re-click can remove just that line (collapse one
    // contributor without disturbing the others).
    _addCdcConvergenceLine(targetCard, chip, clock, fromPin) {
        let block = targetCard.querySelector(
            '[data-cdc-convergence]');
        if (!block) {
            block = document.createElement('div');
            block.dataset.cdcConvergence = '1';
            block.style.cssText
                = 'padding:3px 8px;font-size:11px;'
                + 'background:rgba(80, 140, 220, 0.15);'
                + 'color:var(--fg-primary);'
                + 'border-top:1px dashed rgba(80, 140, 220, 0.6);';
            const head = document.createElement('div');
            head.textContent = '↑ also reached via:';
            head.style.cssText
                = 'font-style:italic;color:var(--fg-muted);'
                + 'margin-bottom:2px;';
            block.appendChild(head);
            targetCard.appendChild(block);
        }
        const line = document.createElement('div');
        line.style.cssText = 'padding-left:10px;';
        line.textContent = `clock ${clock} (from ${fromPin})`;
        block.appendChild(line);
        chip._cdcConvergenceLine = line;
    }

    // Brief outline flash + scrollIntoView so the user sees where
    // their click "landed" when convergence rerouted them to an
    // existing card. Uses a short setTimeout instead of CSS
    // transitions to keep the implementation jsdom-testable.
    _flashCdcCard(card) {
        const orig = card.style.boxShadow || '';
        card.style.boxShadow
            = '0 0 0 3px var(--accent-tab)';
        setTimeout(() => {
            card.style.boxShadow = orig;
        }, 800);
        if (typeof card.scrollIntoView === 'function') {
            try {
                card.scrollIntoView(
                    { behavior: 'smooth', block: 'center' });
            } catch (_) {
                // Some browsers / jsdom don't support the options
                // form; the no-arg call is widely available.
                card.scrollIntoView();
            }
        }
    }

    async _toggleCdcFanIn(pinRef, clock, chip, pinKind, pinName) {
        const key = this._cdcFanInCacheKey(pinRef, clock);
        if (!key || !chip) return;
        // Locate the originating stage card. The chip is a child of
        // the pin row inside the card; walking up to `[data-cdc-
        // stage-card]` gives us the insertion anchor.
        const card = chip.closest('[data-cdc-stage-card]');
        if (!card || !card.parentElement) return;
        const parent = card.parentElement;
        // If an expansion already exists for this (pin, clock), the
        // click is a collapse: remove the wrapper and clear chip
        // styling. We scan via dataset rather than an attribute
        // selector because the key contains `|` characters that
        // would need CSS.escape (not available everywhere).
        const existing = Array.from(
            parent.querySelectorAll('[data-cdc-fan-in-key]'))
            .find(el => el.dataset.cdcFanInKey === key);
        if (existing) {
            const oldStrip = existing.parentElement;
            existing.remove();
            chip.style.outline = '';
            // If this was the only wrapper in its strip, drop the
            // empty strip too so it doesn't leave whitespace
            // above the card.
            if (oldStrip
                && oldStrip.dataset.cdcFanInStrip === '1'
                && oldStrip.children.length === 0) {
                oldStrip.remove();
            }
            return;
        }

        // Disable the chip while the fetch is in flight so a fast
        // double-click doesn't fire two parallel RPCs.
        chip.style.outline = '1px dashed var(--accent-tab)';

        // Fetch from cache or via the backend.
        if (!this._cdcFanInCache) this._cdcFanInCache = new Map();
        let stages = this._cdcFanInCache.get(key);
        if (!stages) {
            try {
                const data = await this._requestWithTimeout({
                    type: 'cdc_pin_fan_in',
                    pin_odb_type: pinRef.odb_type,
                    pin_odb_id:   pinRef.odb_id,
                    clock,
                    mode: this._cdcActiveModeName(this._cdcOverview),
                });
                stages = (data && data.stages) || [];
                this._cdcFanInCache.set(key, stages);
            } catch (e) {
                console.warn('[CDC] fan-in fetch failed', e);
                chip.style.outline = '';
                return;
            }
        }

        // Convergence detection (was here): when a new walk
        // terminated at an FF already on the diagram, we used to
        // skip the new wrapper and add an "also reached via" line
        // to the existing card. User feedback: clicking expects a
        // visible expansion; the annotation is hard to read and
        // hides what's happening. With side-by-side wrappers,
        // duplicates are easy to compare visually, so we just
        // render every walk as its own wrapper. Multi-clock-CK
        // FFs naturally surface as N side-by-side wrappers all
        // ending at the same launch FF — the visual is what
        // tells the user "they all came from here."

        // Render ancestor stages above the originating card. We
        // place each per-(pin, clock) trace into its own wrapper,
        // and lay multiple wrappers SIDE-BY-SIDE in a horizontal
        // "strip" that sits directly above the originating card.
        // Stacking expansions vertically used to make it hard to
        // see what was going on when several were open at once.
        // The strip is created on first wrapper insertion and
        // removed when the last wrapper closes.
        const wrapper = document.createElement('div');
        wrapper.dataset.cdcFanInKey = key;
        wrapper.style.cssText
            = 'display:flex;flex-direction:column;gap:4px;'
            + 'padding:4px 6px;border:1px dashed var(--border);'
            + 'border-radius:3px;'
            + 'min-width:240px;max-width:340px;'
            + 'flex:0 0 auto;'
            + `background:${this._cdcClockTint(clock, 0.06) || ''};`;

        const head = document.createElement('div');
        head.style.cssText
            = 'display:flex;align-items:baseline;gap:8px;'
            + 'font-size:11px;color:var(--fg-muted);';
        const headLabel = document.createElement('span');
        headLabel.textContent = `↑ fan-in trace · clock ${clock}`;
        head.appendChild(headLabel);
        const collapse = document.createElement('a');
        collapse.textContent = '× collapse';
        collapse.style.cssText
            = 'cursor:pointer;color:var(--accent-tab);'
            + 'text-decoration:underline;margin-left:auto;';
        collapse.addEventListener('click', () => {
            const strip = wrapper.parentElement;
            wrapper.remove();
            chip.style.outline = '';
            if (strip
                && strip.dataset.cdcFanInStrip === '1'
                && strip.children.length === 0) {
                strip.remove();
            }
        });
        head.appendChild(collapse);
        wrapper.appendChild(head);

        if (stages.length === 0) {
            const empty = document.createElement('div');
            empty.style.cssText
                = 'padding:6px 8px;font-style:italic;'
                + 'color:var(--fg-muted);';
            empty.textContent = 'No further fan-in — the trace '
                + 'reached a top-level port, a feedback cycle, or '
                + 'a cell with no comb input.';
            wrapper.appendChild(empty);
        } else {
            // Render each ancestor stage with the existing card
            // builder. `syncOrdinal=0`, no syncChain, no
            // hasDownstream — these are pure ancestry stages.
            for (let i = 0; i < stages.length; ++i) {
                if (i > 0) {
                    const prev = stages[i - 1];
                    wrapper.appendChild(this._renderCdcInterStageGap(
                        prev && prev.out_net,
                        (prev && prev.passthroughs_after) || []));
                }
                wrapper.appendChild(this._renderCdcStageCard(
                    stages[i], 0, /*launch=*/clock,
                    /*capture=*/clock,
                    { kind: 'none', depth: 0 },
                    // `disableFanInChips` keeps clock chips in the
                    // expanded ancestor cards visible-but-inert.
                    // Chained re-expansion from within an
                    // expansion was reported as confusing — the
                    // user expects a single trace per chip click,
                    // not a recursive cascade.
                    //
                    // `clockPathContext` flips the multi-clock
                    // convergence semantic: when the trace
                    // started from a CK pin, the converging gate
                    // is a clock-mux, not a data-domain mix.
                    { hasDownstream: false,
                      disableFanInChips: true,
                      clockPathContext: pinKind === 'clock' }));
            }
            // Connector arrow from the last ancestor down to the
            // originating card so the temporal direction is clear.
            const arrow = document.createElement('div');
            arrow.style.cssText
                = 'text-align:center;font-size:11px;'
                + 'color:var(--fg-muted);padding:2px;';
            arrow.textContent = '↓ feeds into';
            wrapper.appendChild(arrow);
        }

        // Side-by-side layout: insert this wrapper into the strip
        // sitting directly above the originating card. Create the
        // strip if it doesn't exist yet.
        let strip = card.previousElementSibling;
        if (!strip
            || !strip.dataset
            || strip.dataset.cdcFanInStrip !== '1') {
            strip = document.createElement('div');
            strip.dataset.cdcFanInStrip = '1';
            strip.style.cssText
                = 'display:flex;flex-direction:row;gap:8px;'
                + 'align-items:stretch;'
                + 'overflow-x:auto;margin-bottom:4px;';
            parent.insertBefore(strip, card);
        }
        strip.appendChild(wrapper);
    }

    // Clock-mix tracer (replaces per-chip data fan-in). Triggered
    // from the `↑ trace mix` button on a multi-clock pin row. The
    // backend returns a TREE rooted at the originating pin: each
    // mixer node has per-input branches that recurse upstream to
    // ports / registers / depth-limit terminals. The frontend
    // renders the tree as nested branch columns converging at each
    // mixer card, with the originating pin at the bottom.
    // Toggle the clock-mix tracer on a pin.
    //
    // `opts.anchor` (default: btn.closest('[data-cdc-stage-card]'))
    //   The DOM element the trace strip attaches relative to. The
    //   path-detail-card use case finds the surrounding stage card
    //   via the data-attribute selector; the mix-endpoint listing
    //   passes its row element directly.
    // `opts.placement` (default: 'before')
    //   'before' inserts the strip ABOVE the anchor (path-detail
    //   convention — the trace tree feeds DOWN into the card).
    //   'after' inserts it BELOW (listing convention — the trace
    //   appears as an in-place expansion of the row).
    async _toggleCdcClockMix(pinRef, clocks, btn, pinKind, pinName, opts) {
        opts = opts || {};
        const clockList = Array.isArray(clocks)
            ? clocks.filter(c => c).slice() : [];
        if (!pinRef || !clockList.length || !btn) return;
        // Cache / collapse key — pin id + sorted clock set so two
        // walks for the same pin with different clock sets stay
        // separate (manual recursion entry from a contributing
        // input narrows the clock set; each gets its own wrapper).
        const sorted = clockList.slice().sort();
        const key = `mix|${pinRef.odb_type}|${pinRef.odb_id}|`
            + sorted.join(',');
        const card = opts.anchor
            || btn.closest('[data-cdc-stage-card]');
        if (!card || !card.parentElement) return;
        const parent = card.parentElement;
        const placement = opts.placement === 'after' ? 'after' : 'before';
        const existing = Array.from(
            parent.querySelectorAll('[data-cdc-fan-in-key]'))
            .find(el => el.dataset.cdcFanInKey === key);
        if (existing) {
            const oldStrip = existing.parentElement;
            existing.remove();
            btn.style.outline = '';
            if (oldStrip
                && oldStrip.dataset.cdcFanInStrip === '1'
                && oldStrip.children.length === 0) {
                oldStrip.remove();
            }
            return;
        }
        btn.style.outline = '1px dashed var(--accent-tab)';

        if (!this._cdcClockMixCache) this._cdcClockMixCache = new Map();
        let tree;
        if (this._cdcClockMixCache.has(key)) {
            tree = this._cdcClockMixCache.get(key);
        } else {
            try {
                const data = await this._requestWithTimeout({
                    type: 'cdc_clock_mix_trace',
                    pin_odb_type: pinRef.odb_type,
                    pin_odb_id:   pinRef.odb_id,
                    clocks: clockList,
                    mode: this._cdcActiveModeName(this._cdcOverview),
                });
                tree = (data && data.tree) || null;
                this._cdcClockMixCache.set(key, tree);
            } catch (e) {
                console.warn('[CDC] clock-mix fetch failed', e);
                btn.style.outline = '';
                return;
            }
        }

        const wrapper = document.createElement('div');
        wrapper.dataset.cdcFanInKey = key;
        wrapper.dataset.cdcClockMixWrapper = '1';
        wrapper.style.cssText
            = 'display:flex;flex-direction:column;gap:6px;'
            + 'padding:6px 8px;border:1px dashed var(--border);'
            + 'border-radius:3px;'
            + 'flex:0 0 auto;'
            + 'min-width:260px;';

        const head = document.createElement('div');
        head.style.cssText
            = 'display:flex;align-items:baseline;gap:8px;'
            + 'font-size:11px;color:var(--fg-muted);';
        const headLabel = document.createElement('span');
        headLabel.textContent
            = `↑ clock-mix trace · ${clockList.join(', ')}`;
        head.appendChild(headLabel);
        const collapse = document.createElement('a');
        collapse.textContent = '× collapse';
        collapse.style.cssText
            = 'cursor:pointer;color:var(--accent-tab);'
            + 'text-decoration:underline;margin-left:auto;';
        collapse.addEventListener('click', () => {
            const strip = wrapper.parentElement;
            wrapper.remove();
            btn.style.outline = '';
            if (strip
                && strip.dataset.cdcFanInStrip === '1'
                && strip.children.length === 0) {
                strip.remove();
            }
        });
        head.appendChild(collapse);
        wrapper.appendChild(head);

        if (!tree) {
            const empty = document.createElement('div');
            empty.style.cssText
                = 'padding:6px 8px;font-style:italic;'
                + 'color:var(--fg-muted);';
            empty.textContent = 'No upstream driver — pin is a top-'
                + 'level port or has no resolvable driver.';
            wrapper.appendChild(empty);
        } else {
            wrapper.appendChild(this._renderCdcClockMixNode(tree));
            const arrow = document.createElement('div');
            arrow.style.cssText
                = 'text-align:center;font-size:11px;'
                + 'color:var(--fg-muted);padding:2px;';
            arrow.textContent = '↓ feeds into';
            wrapper.appendChild(arrow);
        }

        // Strip placement: BEFORE the anchor (path-detail convention,
        // trace tree above the card) or AFTER (listing convention,
        // trace appears as an in-place row expansion).
        let strip = placement === 'after'
            ? card.nextElementSibling
            : card.previousElementSibling;
        if (!strip
            || !strip.dataset
            || strip.dataset.cdcFanInStrip !== '1') {
            strip = document.createElement('div');
            strip.dataset.cdcFanInStrip = '1';
            strip.style.cssText
                = 'display:flex;flex-direction:row;gap:8px;'
                + (placement === 'after'
                    ? 'align-items:flex-start;margin-top:4px;'
                    : 'align-items:flex-end;margin-bottom:4px;')
                + 'overflow-x:auto;';
            if (placement === 'after') {
                parent.insertBefore(strip, card.nextSibling);
            } else {
                parent.insertBefore(strip, card);
            }
        }
        strip.appendChild(wrapper);
    }

    // Recursive entry. Renders the subtree(s) above this node, then
    // the node's own card. Returns a column-flex DOM element with
    // upstream content on top and the node's card at the bottom.
    _renderCdcClockMixNode(node) {
        if (!node) return document.createElement('div');
        const card = this._renderCdcClockMixCard(node);
        const above = this._renderCdcClockMixSubtree(node);
        if (!above) {
            return card;
        }
        const container = document.createElement('div');
        container.style.cssText
            = 'display:flex;flex-direction:column;'
            + 'align-items:stretch;gap:6px;';
        container.appendChild(above);
        container.appendChild(card);
        return container;
    }

    // Subtree for a node — DOM that goes ABOVE this node's card.
    //   - Mixer / NetMixer: a row of branch columns. Each column
    //     recurses on the branch's subtree. Branches lay out left-
    //     to-right, aligned at the bottom (each branch's terminal
    //     sits at the top, the column's deepest card sits just
    //     above the parent mixer).
    //   - RegisterTransit / CombTransit: a single recursed subtree
    //     above the transit card (linear chain; no branching).
    //   - Terminal kinds: nothing above.
    _renderCdcClockMixSubtree(node) {
        if (node.kind === 'mixer' || node.kind === 'net_mixer') {
            const branches = (node.branches || []).filter(b => b);
            if (!branches.length) return null;
            const row = document.createElement('div');
            // `min-width:max-content` makes the row claim its full
            // natural width (N * 320px + gaps). Without it, when the
            // total exceeds the strip's visible width, the row gets
            // squeezed by its parent's stretch and the fixed-width
            // columns inside (which have flex-shrink:0) overflow,
            // visually overlapping. With it, the row holds its size,
            // wrapper grows to fit, and the strip's overflow-x:auto
            // takes over for horizontal scrolling.
            row.style.cssText
                = 'display:flex;flex-direction:row;gap:8px;'
                + 'align-items:flex-end;justify-content:center;'
                + 'min-width:max-content;';
            for (const branch of branches) {
                const col = document.createElement('div');
                // 320px minimum — leaf branches with single cards
                // line up cleanly at that width. Allowed to grow
                // (`flex:0 0 auto`) so a branch carrying its own
                // nested mixer can host its sub-row without the
                // sub-row's content overflowing the column and
                // visually colliding with sibling branches. Two
                // siblings at the same level may end up different
                // widths if one is deeper, but nothing overlaps.
                col.style.cssText
                    = 'display:flex;flex-direction:column;gap:6px;'
                    + 'flex:0 0 auto;min-width:320px;'
                    + 'align-items:stretch;';
                if (branch.subtree) {
                    col.appendChild(this._renderCdcClockMixNode(
                        branch.subtree));
                }
                // "feeds into <pin>" label between this column and the
                // mixer card below: surfaces which input pin of the
                // downstream gate this branch wires to. The MIX card's
                // body lists the same info per-input, but a connector
                // label here matches the visual flow (column → arrow
                // → that input pin on the mixer).
                if (branch.pin) {
                    const feeds = document.createElement('div');
                    feeds.style.cssText
                        = 'text-align:center;font-size:11px;'
                        + 'color:var(--fg-muted);padding:1px 4px;'
                        + 'min-width:0;' + TRUNCATE_PATH_CSS;
                    const leaf = this._pinLeafName(
                        branch.pin, node.instance || null);
                    feeds.textContent = `↓ feeds into ${leaf}`;
                    feeds.title = `feeds into ${branch.pin}`;
                    col.appendChild(feeds);
                }
                row.appendChild(col);
            }
            return row;
        }
        if (node.kind === 'register_transit'
            || node.kind === 'comb_transit') {
            if (node.child) {
                return this._renderCdcClockMixNode(node.child);
            }
        }
        return null;
    }

    // One node's card. Shapes:
    //   - mixer / net_mixer  ⚡ banner; data-path or clock-path tint
    //                        based on `on_clock_path`. Body lists OUT
    //                        + cur_clocks chips; contributors are
    //                        already rendered as the branch columns
    //                        above so we don't duplicate them here.
    //   - register_transit /
    //     comb_transit       collapsed by default — single-line
    //                        `↑ via {instance}`. Click expands to
    //                        full body (master cell, pin rows,
    //                        clock chips).
    //   - port               ⏚ banner. Body shows port pin +
    //                        actual_clocks chips. Unaccounted
    //                        warning band when applicable.
    //   - stuck              ⊘ banner. Body shows pin + actual
    //                        clocks chips + unaccounted warning.
    //   - feedback           ↺ banner.
    //   - depth_limit        ⋯ banner + ↑ continue trace link.
    //   - unresolved         ? banner.
    _renderCdcClockMixCard(node) {
        const card = document.createElement('div');
        card.dataset.cdcStageCard = '1';
        card.dataset.cdcMixStageKind = node.kind || 'unresolved';
        if (node.on_clock_path) {
            card.dataset.cdcMixOnClockPath = '1';
        }
        card.style.cssText =
            'border:1px solid var(--border);border-radius:4px;' +
            'background:var(--bg-input);font-family:monospace;' +
            'font-size:12px;min-width:0;overflow:hidden;'
            // `width:100%` + `contain:inline-size` means the card's
            // width is dictated by its parent (320px branch column,
            // or the trunk wrapper's natural span = the row above),
            // and the card's own content (including a long instance
            // path in its header) does NOT contribute to the parent's
            // intrinsic-width calculation. Without containment, a
            // long instance name on the trunk MIX card pushed the
            // wrapper wider than the row above; with it, the trunk
            // card lines up exactly with the row width.
            + 'flex:0 0 auto;width:100%;contain:inline-size;';

        // Transit cards are collapsed-by-default. Render only a
        // one-line summary (instance name) until clicked.
        if (node.kind === 'register_transit'
            || node.kind === 'comb_transit') {
            this._buildCdcMixTransitCard(card, node);
            return card;
        }

        // All other kinds get the full banner+header+body layout.
        const banner = document.createElement('div');
        banner.style.cssText
            = 'padding:3px 8px;font-size:11px;font-weight:600;'
            + 'border-bottom:1px solid var(--border);';
        let bannerText = '';
        let bannerBg = 'var(--bg-header)';
        switch (node.kind) {
            case 'mixer':
                if (node.on_clock_path) {
                    bannerText = node.degenerate
                        ? '⚡ CLOCK-PATH MIX · single contributor'
                        : `⚡ CLOCK-PATH MIX · ${node.branches.length}`
                          + ' inputs merge';
                    // Yellow/orange — clock convergence is often
                    // intentional (clock mux), not a bug per se.
                    bannerBg = 'rgba(255, 167, 38, 0.20)';
                } else {
                    bannerText = node.degenerate
                        ? '⚠ DATA-PATH MIX · single contributor'
                        : `⚠ DATA-PATH MIX · ${node.branches.length}`
                          + ' inputs merge';
                    // Red — data-path mix is the actual CDC bug.
                    bannerBg = 'rgba(220, 64, 64, 0.22)';
                }
                break;
            case 'net_mixer':
                if (node.on_clock_path) {
                    bannerText = node.degenerate
                        ? '⚡ CLOCK-PATH NET CONVERGENCE · 1 driver'
                        : `⚡ CLOCK-PATH NET CONVERGENCE · `
                          + `${node.branches.length} drivers`;
                    bannerBg = 'rgba(255, 167, 38, 0.20)';
                } else {
                    bannerText = node.degenerate
                        ? '⚠ DATA-PATH NET CONVERGENCE · 1 driver'
                        : `⚠ DATA-PATH NET CONVERGENCE · `
                          + `${node.branches.length} drivers`;
                    bannerBg = 'rgba(220, 64, 64, 0.22)';
                }
                break;
            case 'port':
                bannerText = '⏚ clock source · port';
                bannerBg = 'rgba(76, 175, 80, 0.18)';
                break;
            case 'stuck':
                bannerText = '⊘ trace stopped';
                bannerBg = 'rgba(220, 64, 64, 0.18)';
                break;
            case 'feedback':
                bannerText = '↺ feedback cycle';
                bannerBg = 'rgba(220, 64, 64, 0.12)';
                break;
            case 'depth_limit':
                bannerText
                    = '⋯ depth limit reached · click to continue';
                bannerBg = 'var(--bg-header)';
                break;
            default:
                bannerText = '? unresolved driver';
                bannerBg = 'var(--bg-header)';
        }
        banner.style.background = bannerBg;
        banner.textContent = bannerText;
        card.appendChild(banner);

        if (node.instance || node.cell) {
            const head = document.createElement('div');
            head.style.cssText
                = 'padding:3px 8px;display:flex;'
                + 'justify-content:space-between;gap:8px;'
                + 'align-items:baseline;'
                + 'border-bottom:1px solid var(--border);';
            const instEl = document.createElement('span');
            instEl.style.cssText
                = 'color:var(--fg-primary);min-width:0;'
                + TRUNCATE_PATH_CSS;
            instEl.textContent = node.instance || '';
            instEl.title = node.instance || '';
            head.appendChild(instEl);
            if (node.cell) {
                const cellEl = document.createElement('span');
                cellEl.style.cssText
                    = 'color:var(--fg-muted);font-size:11px;'
                    + 'flex:0 0 auto;white-space:nowrap;';
                cellEl.textContent = node.cell;
                head.appendChild(cellEl);
            }
            card.appendChild(head);
        }

        const body = document.createElement('div');
        body.style.cssText = 'padding:4px 8px;';

        const pinRow = (label, pinPath, clocks) => {
            const row = document.createElement('div');
            row.style.cssText
                = 'display:flex;align-items:baseline;gap:6px;'
                + 'padding:1px 0;font-size:12px;';
            const lab = document.createElement('span');
            lab.textContent = label;
            lab.style.cssText
                = 'color:var(--fg-muted);font-weight:600;'
                + 'min-width:36px;';
            row.appendChild(lab);
            const path = document.createElement('span');
            path.style.cssText
                = 'color:var(--fg-primary);min-width:0;flex:1 1 auto;'
                + TRUNCATE_PATH_CSS;
            path.textContent
                = pinPath
                    ? this._pinLeafName(pinPath, node.instance || null)
                    : '—';
            path.title = pinPath || '';
            row.appendChild(path);
            const chips = document.createElement('span');
            chips.style.cssText
                = 'display:inline-flex;gap:3px;flex-wrap:wrap;'
                + 'align-items:center;';
            for (const c of (clocks || [])) {
                chips.appendChild(this._cdcMakeClockChip(c));
            }
            row.appendChild(chips);
            body.appendChild(row);
        };

        if (node.kind === 'mixer' || node.kind === 'net_mixer') {
            // OUT + the merged clock set is the headline. The per-
            // input pin info is conveyed by the branch columns above
            // the card plus their `↓ feeds into <pin>` labels — no
            // need to duplicate it as rows on the merger card body
            // (doing so made multi-clock mixers very tall, since each
            // row's chip cluster wraps to multiple lines).
            if (node.out_pin) {
                pinRow('OUT', node.out_pin, node.clocks || []);
            } else if ((node.clocks || []).length) {
                pinRow('CLK', '—', node.clocks);
            }
        } else if (node.kind === 'port') {
            pinRow('PORT', node.out_pin || '—',
                   node.actual_clocks && node.actual_clocks.length
                       ? node.actual_clocks
                       : node.clocks || []);
        } else if (node.kind === 'stuck') {
            if (node.via_pin) {
                pinRow('VIA', node.via_pin,
                       node.actual_clocks || []);
            } else if (node.out_pin) {
                pinRow('OUT', node.out_pin,
                       node.actual_clocks || node.clocks || []);
            }
        } else if (node.kind === 'depth_limit') {
            // No body row: the followed-pin name was redundant
            // jargon ("· From <pin>") that didn't help users decide
            // anything. The continue-trace link below carries the
            // pin context implicitly (clicking starts the next
            // walk from there).
            if (node.via_pin && node.via_pin_odb_type
                && node.via_pin_odb_id != null) {
                const cont = document.createElement('div');
                cont.style.cssText
                    = 'padding:4px 8px;border-top:1px solid var(--border);'
                    + 'text-align:center;';
                const link = document.createElement('a');
                link.textContent = '↑ continue trace';
                link.dataset.cdcTraceMix = '1';
                link.style.cssText
                    = 'cursor:pointer;color:var(--accent-tab);'
                    + 'text-decoration:underline;'
                    + 'text-decoration-style:dotted;'
                    + 'font-size:11px;white-space:nowrap;';
                link.title
                    = 'Walk another batch of stages upstream from '
                    + 'where the depth cap was hit.';
                const stageClocks = (node.clocks || []).slice();
                link.addEventListener('click', () => {
                    this._toggleCdcClockMix(
                        { odb_type: node.via_pin_odb_type,
                          odb_id: node.via_pin_odb_id },
                        stageClocks,
                        link,
                        'data',
                        node.via_pin);
                    // Once the upstream trace has been launched,
                    // this depth-limit terminal is just visual
                    // noise — the new wrapper carries everything
                    // the user needs. Hide the card so the column
                    // ends at its real upstream chain.
                    card.style.display = 'none';
                });
                cont.appendChild(link);
                body.appendChild(cont);
            }
        } else {
            if (node.out_pin) {
                pinRow('PIN', node.out_pin, node.clocks || []);
            } else if ((node.clocks || []).length) {
                pinRow('CLK', '—', node.clocks);
            }
        }
        card.appendChild(body);

        // Unaccounted-clocks warning band. When a terminal's
        // actual_clocks doesn't cover the cur_clocks the walker was
        // tracing, the missing clocks reach cur_pin via some path
        // the walker didn't follow. Surface them prominently so the
        // user knows the trace on this branch is incomplete.
        if (node.unaccounted_clocks
            && node.unaccounted_clocks.length) {
            const warn = document.createElement('div');
            warn.style.cssText
                = 'padding:3px 8px;font-size:11px;'
                + 'background:rgba(255, 167, 38, 0.20);'
                + 'color:var(--fg-primary);'
                + 'border-top:1px solid var(--border);';
            warn.textContent
                = `⚠ ${node.unaccounted_clocks.length} clock`
                + (node.unaccounted_clocks.length === 1 ? '' : 's')
                + ' unaccounted for here: '
                + node.unaccounted_clocks.join(', ');
            warn.title
                = 'These clocks reach the originating pin via some '
                + 'route this branch didn\'t take. Look at sibling '
                + 'branches for where they come from.';
            card.appendChild(warn);
        }
        return card;
    }

    // Transit card — collapsed by default to a single line
    // `↑ via {instance}`. Click on the summary line expands the
    // body to reveal the master cell, pin rows, and clock chips.
    // Per user feedback 2026-05-02 round 4: transit cards are
    // useful for verifying the path but noisy when always
    // expanded; instance name in the header is the minimum useful
    // info.
    _buildCdcMixTransitCard(card, node) {
        const verb = node.kind === 'register_transit'
            ? 'via CK of'
            : 'via';
        const instLeaf = (node.instance || '')
            .split('/')
            .pop() || '(unknown)';

        const summary = document.createElement('a');
        summary.dataset.cdcMixTransitSummary = '1';
        summary.style.cssText
            = 'display:flex;align-items:center;gap:6px;'
            + 'cursor:pointer;'
            + 'padding:4px 8px;font-size:12px;'
            + 'color:var(--fg-primary);'
            // min-width:0 lets the `head` span (with TRUNCATE_PATH_CSS)
            // actually shrink and leading-ellipsize so a long
            // `↑ via <leafname>` line doesn't push the card wider.
            + 'min-width:0;';
        const arrow = document.createElement('span');
        arrow.style.cssText
            = 'color:var(--accent-tab);font-size:11px;'
            + 'flex:0 0 auto;';
        const head = document.createElement('span');
        head.style.cssText
            = 'flex:1 1 auto;min-width:0;'
            + TRUNCATE_PATH_CSS;
        head.title = node.instance || '';
        summary.appendChild(arrow);
        summary.appendChild(head);

        const inner = document.createElement('div');
        inner.style.cssText
            = 'border-top:1px solid var(--border);'
            + 'padding:4px 8px;display:none;'
            + 'flex-direction:column;gap:2px;';

        const bodyHeader = document.createElement('div');
        bodyHeader.style.cssText
            = 'display:flex;justify-content:space-between;'
            + 'gap:8px;align-items:baseline;'
            + 'font-size:11px;color:var(--fg-muted);'
            + 'padding-bottom:2px;'
            // min-width:0 lets bodyHeadInst (with TRUNCATE_PATH_CSS)
            // leading-ellipsize within the card's fixed width when
            // the body is expanded.
            + 'min-width:0;';
        const bodyHeadInst = document.createElement('span');
        bodyHeadInst.textContent = node.instance || '';
        bodyHeadInst.title = node.instance || '';
        bodyHeadInst.style.cssText
            = 'color:var(--fg-secondary);min-width:0;'
            + TRUNCATE_PATH_CSS;
        bodyHeader.appendChild(bodyHeadInst);
        if (node.cell) {
            const cellEl = document.createElement('span');
            cellEl.textContent = node.cell;
            cellEl.style.cssText
                = 'flex:0 0 auto;white-space:nowrap;';
            bodyHeader.appendChild(cellEl);
        }
        inner.appendChild(bodyHeader);

        const pinRow = (label, pinPath, clocks) => {
            const row = document.createElement('div');
            row.style.cssText
                = 'display:flex;align-items:baseline;gap:6px;'
                + 'padding:1px 0;font-size:12px;';
            const lab = document.createElement('span');
            lab.textContent = label;
            lab.style.cssText
                = 'color:var(--fg-muted);font-weight:600;'
                + 'min-width:36px;';
            row.appendChild(lab);
            const path = document.createElement('span');
            path.style.cssText
                = 'color:var(--fg-primary);min-width:0;flex:1 1 auto;'
                + TRUNCATE_PATH_CSS;
            path.textContent
                = pinPath
                    ? this._pinLeafName(pinPath, node.instance || null)
                    : '—';
            path.title = pinPath || '';
            row.appendChild(path);
            const chips = document.createElement('span');
            chips.style.cssText
                = 'display:inline-flex;gap:3px;flex-wrap:wrap;'
                + 'align-items:center;';
            for (const c of (clocks || [])) {
                chips.appendChild(this._cdcMakeClockChip(c));
            }
            row.appendChild(chips);
            inner.appendChild(row);
        };

        if (node.kind === 'register_transit') {
            if (node.out_pin) {
                pinRow('Q', node.out_pin, node.clocks || []);
            }
            if (node.via_pin) {
                pinRow('CK', node.via_pin, node.clocks || []);
            }
        } else {
            if (node.out_pin) {
                pinRow('OUT', node.out_pin, node.clocks || []);
            }
            if (node.via_pin) {
                pinRow('IN', node.via_pin, node.clocks || []);
            }
        }

        const setLabel = (open) => {
            arrow.textContent = open ? '▼' : '▶';
            head.textContent = `↑ ${verb} ${instLeaf}`;
        };
        setLabel(false);
        summary.addEventListener('click', () => {
            const open = inner.style.display !== 'none';
            inner.style.display = open ? 'none' : 'flex';
            setLabel(!open);
        });
        card.appendChild(summary);
        card.appendChild(inner);
    }

    // Tinted clock chip used inside the clock-mix renderer. Same
    // hash-to-hue background as the rest of the diagram so the
    // legend stays consistent (user feedback 2026-05-02 round 3).
    _cdcMakeClockChip(name) {
        const chip = document.createElement('span');
        chip.textContent = name;
        chip.dataset.cdcClockChip = name;
        const tint = this._cdcClockTint(name, 0.30);
        chip.style.cssText
            = 'padding:0 6px;border-radius:8px;font-size:11px;'
            + `background:${tint || 'var(--bg-header)'};`
            + 'color:var(--fg-primary);white-space:nowrap;';
        return chip;
    }


    // One stage card. Three shapes depending on `s.kind`:
    //   - "register"  flop — D + Q pin rows; banner says LAUNCH /
    //                 STAGE 1 / STAGE 2+ / CROSSOVER depending on
    //                 which side of the crossover this stage sits on
    //                 and whether a sync chain was detected.
    //   - "comb"      combinational gate — IN + OUT pin rows + any
    //                 fan-in pins listed below.
    //   - "port"      top-level INPUT port acting as the launch source.
    //
    // `syncChain` is `data.sync_chain` from cdc_path_detail — its
    // `kind` decides whether the capture flop is part of a sync chain
    // (banner: "Stage 1", green) or a real CDC bug (banner:
    // "⚠ Crossover", red). Subsequent sync stages always banner as
    // "Stage 2+" — the actual count is in the path-list cell.
    //
    // The card body shades by clock domain (`s.clock`) so users can
    // see the launch → capture flip visually as they scan down the
    // diagram — same clock name produces the same hue every time.
    _renderCdcStageCard(s, syncOrdinal, launch, capture, syncChain,
                        opts) {
        const card = document.createElement('div');
        // `data-cdc-stage-card` marks this as a stage card so the
        // fan-in expansion handler can locate it via `chip.closest`
        // when a clock chip in the pin rows below is clicked.
        // `data-cdc-stage-instance` carries the stage's instance
        // path so the convergence detector can look up "is this
        // FF already on screen?" without walking textContent.
        // `data-cdc-stage-terminal-pin` is the stage's
        // "exit" pin (Q for registers, the port itself for top-
        // level ports) — convergence merging keys on BOTH the
        // instance AND this pin so two walks landing at different
        // pins of the same instance stay separate.
        card.dataset.cdcStageCard = '1';
        if (s && s.instance) {
            card.dataset.cdcStageInstance = s.instance;
        }
        const terminalPin = (s && (s.q_pin || s.out_pin)) || null;
        if (terminalPin) {
            card.dataset.cdcStageTerminalPin = terminalPin;
        }
        card.style.cssText =
            'border:1px solid var(--border);border-radius:4px;' +
            'background:var(--bg-input);font-family:monospace;font-size:12px;' +
            'min-width:0;overflow:hidden;';

        // Crossover flop is the bug-relevant stage; sync stages are the
        // mitigation; launch register is the source. Border colour
        // anchors the eye on the crossover first.
        if (s.is_capture) {
            card.style.borderColor = 'rgba(220, 64, 64, 0.7)';
        } else if (s.is_sync_stage) {
            card.style.borderColor = 'rgba(76, 175, 80, 0.7)';
        } else if (s.is_launch) {
            card.style.borderColor = 'rgba(255, 167, 38, 0.7)';
        }

        // Clock-domain shading on the card body. Shading is applied
        // BELOW the banner so the banner's stronger role colour stays
        // legible; the body tint is what tells the user "this stage
        // lives in clock domain X" without having to read the badge.
        const tintClock = s.clock || s.capture_clock;
        const tint = this._cdcClockTint(tintClock, 0.10);

        // ── Role banner ────────────────────────────────────────────
        const banner = document.createElement('div');
        banner.style.cssText =
            'padding:3px 8px;font-size:11px;letter-spacing:0.6px;' +
            'text-transform:uppercase;font-weight:600;';
        // The capture flop's role depends on whether a sync chain was
        // detected. If it IS the first flop of a synchronizer chain,
        // call it "Stage 1" (green) — it's the meta-resolver doing
        // its job. If no chain was detected, it's the actual CDC
        // crossover bug — show "⚠ Crossover" (red).
        const syncKind = (syncChain && syncChain.kind) || 'none';
        const partOfSync = syncKind === 'ff_chain'
            || syncKind === 'liberty_sync'
            || syncKind === 'composite'
            || syncKind === 'whitelisted';
        const hasDownstream = !!(opts && opts.hasDownstream);
        if (s.is_capture && partOfSync) {
            // The capture flop IS the crossover (where launch and
            // capture clocks meet on D vs CK). When the chain has at
            // least one downstream sync stage, this flop is also
            // "stage 1" of that chain (the meta-resolver) — say so
            // explicitly. When the chain is single-celled (a SYNC
            // cell on its own, or a whitelisted single flop), there
            // is no stage 2+, so calling this "stage 1" would imply
            // a stage that doesn't exist. Drop the suffix in that
            // case but keep the green colour so the user still sees
            // "this is synced".
            card.style.borderColor = 'rgba(76, 175, 80, 0.7)';
            banner.style.background = 'rgba(76, 175, 80, 0.18)';
            banner.style.color = 'var(--fg-primary)';
            if (hasDownstream) {
                banner.textContent = 'crossover · stage 1';
                banner.title =
                    `Crossover + first sync stage. The launch-domain ` +
                    `(${launch}) signal lands on this flop's D pin and ` +
                    `is captured by the capture-domain (${capture}) ` +
                    `clock on its CK — that's the crossover. This flop ` +
                    `is also the meta-resolver: if the input lands during ` +
                    `a transition, metastability is resolved here before ` +
                    `the data is re-sampled by stage 2+.`;
            } else {
                banner.textContent = 'crossover';
                banner.title =
                    `Crossover. The launch-domain (${launch}) signal ` +
                    `lands on this flop's D pin and is captured by the ` +
                    `capture-domain (${capture}) clock on its CK. The ` +
                    `chain is single-celled — synchronized internally ` +
                    `(via this cell's statetable / whitelist), no ` +
                    `external sync stages downstream.`;
            }
        } else if (s.is_capture) {
            banner.style.background = 'rgba(220, 64, 64, 0.20)';
            banner.style.color = 'var(--fg-primary)';
            banner.textContent = '⚠ crossover';
            banner.title =
                `Capture flop — the launch-domain (${launch}) signal ` +
                `lands on this flop's D pin and is captured by the ` +
                `capture-domain (${capture}) clock on its CK. No sync ` +
                `chain detected, so the metastability risk is live at ` +
                `this exact pin.`;
        } else if (s.is_sync_stage) {
            banner.style.background = 'rgba(76, 175, 80, 0.18)';
            banner.style.color = 'var(--fg-primary)';
            // Capture flop is stage 1 (the meta-resolver); subsequent
            // sync stages count up from there. syncOrdinal is computed
            // by the diagram loop as `i - captureIdx` (1 for the
            // first sync stage), so the banner number is one more.
            banner.textContent = `sync stage ${syncOrdinal + 1}`;
            banner.title =
                `Sync stage ${syncOrdinal + 1}. The data has already ` +
                'been re-sampled at least once upstream; this stage ' +
                'adds further margin against metastability ' +
                'propagation. Reached via single-fanout flop-to-flop ' +
                'wiring (buffers and inverters tolerated, no ' +
                'combinational decision logic).';
        } else if (s.is_launch && s.kind === 'port') {
            banner.style.background = 'rgba(255, 167, 38, 0.20)';
            banner.style.color = 'var(--fg-primary)';
            banner.textContent = '◂ launch · port';
            banner.title =
                `Top-level input port — this is where the ${launch} ` +
                'signal enters the design. The launch-side walk could ' +
                'not trace through the module boundary.';
        } else if (s.is_launch) {
            banner.style.background = 'rgba(255, 167, 38, 0.20)';
            banner.style.color = 'var(--fg-primary)';
            banner.textContent = '◂ launch';
            banner.title =
                `Launch flop — this register's CK rides ${launch}, and ` +
                'its Q feeds the combinational/sync chain that ends at ' +
                'the crossover.';
        } else if (s.kind === 'comb' && s.stuck_clock_not_in_inputs) {
            // Backend's strict-clock-match cut the walk here: this
            // gate's inputs none carry the requested clock, so the
            // walker (correctly) refused to follow a wrong input
            // and instead emitted this gate as the terminal stage
            // of the trace. Banner is yellow/amber — neither a
            // success ("we found the source") nor a hard bug
            // ("domain mix") — informational so the user audits
            // the listed input clocks and decides if they really
            // expected the requested clock to reach this gate.
            card.style.borderColor = 'rgba(255, 167, 38, 0.7)';
            banner.style.background = 'rgba(255, 167, 38, 0.20)';
            banner.style.color = 'var(--fg-primary)';
            banner.textContent = `⊘ trace stopped · ${s.cell || 'gate'}`;
            banner.title =
                `Trace stopped at this gate — none of its `
                + 'inputs carry the requested clock. The walker '
                + 'refused to silently follow a wrong-domain '
                + 'input, since that would misattribute the '
                + 'launch source. Inspect the IN / +IN rows '
                + 'below to see each input\'s actual clock '
                + 'domain — if you expected the requested clock '
                + 'to reach here, this is a topology mismatch '
                + 'worth investigating.';
        } else if (s.kind === 'comb' && s.is_domain_mix) {
            // Multi-clock convergence at this gate. Two distinct
            // semantics depending on whether the user is walking
            // a DATA path (the default for path-detail) or a
            // CLOCK path (when the fan-in trace started from a
            // CK pin):
            //   - DATA path: this is a real CDC bug — different
            //     clocks converging on a data signal before it
            //     reaches the capture flop.
            //   - CLOCK path: this is a clock mux / glitchless
            //     clock OR — by design (or accident), multiple
            //     clocks merge on a clock net here. NOT a data
            //     metastability bug; it's a clock-tree topology
            //     question instead.
            // Frontend distinguishes via `opts.clockPathContext`,
            // which `_toggleCdcFanIn` sets when the originating
            // chip lived on a CK pin row.
            const isClockPath = !!(opts && opts.clockPathContext);
            card.style.borderColor = isClockPath
                ? 'rgba(80, 140, 220, 0.7)'
                : 'rgba(220, 64, 64, 0.7)';
            banner.style.background = isClockPath
                ? 'rgba(80, 140, 220, 0.20)'
                : 'rgba(220, 64, 64, 0.20)';
            banner.style.color = 'var(--fg-primary)';
            if (isClockPath) {
                banner.textContent
                    = `⚠ clock-mux convergence · ${s.cell || 'gate'}`;
                banner.title =
                    `Clock-tree convergence at this gate — multiple `
                    + 'clocks merge on a CLOCK net here. Often a '
                    + 'glitchless clock-mux pattern (intentional), '
                    + 'sometimes an unintended clock-OR (a CDC '
                    + 'concern in its own right). Click the +IN rows '
                    + 'to inspect each clock\'s upstream source.';
            } else {
                banner.textContent
                    = `⚠ domain mix · ${s.cell || 'gate'}`;
                banner.title =
                    `Cross-domain mix at this gate — inputs from `
                    + 'multiple clock domains converge here, BEFORE '
                    + 'the capture flop. The capture flop downstream '
                    + 'only re-samples the already-mixed signal; the '
                    + 'actual CDC bug originates on this gate. Click '
                    + 'the +IN rows to inspect the inputs from the '
                    + 'other domain.';
            }
        } else if (s.kind === 'comb') {
            banner.style.background = 'var(--bg-header)';
            banner.style.color = 'var(--fg-secondary)';
            banner.textContent = `comb · ${s.cell || 'gate'}`;
            banner.title =
                'Combinational gate on the launch-side path between ' +
                'the launch flop\'s Q and the capture flop\'s D. The ' +
                'input matching the launch clock is followed for the ' +
                'back-walk; other inputs are listed under fan-in.';
        } else {
            banner.style.background = 'var(--bg-header)';
            banner.style.color = 'var(--fg-muted)';
            banner.textContent = 'stage';
        }
        card.appendChild(banner);

        // ── Instance + cell row ────────────────────────────────────
        // Skipped for port stages: a top-level port's "instance" name
        // and its OUT pin name are the same string, so this row
        // would duplicate the OUT row below. Register / comb stages
        // have an instance distinct from their pin leaves so the row
        // adds real info there.
        if (s.kind !== 'port') {
            const instRow = document.createElement('div');
            instRow.style.cssText =
                'padding:5px 8px;display:flex;align-items:baseline;' +
                'gap:8px;border-bottom:1px solid var(--border-subtle);';
            if (tint) instRow.style.background = tint;
            const instEl = document.createElement('span');
            instEl.style.cssText =
                'font-weight:600;flex:1;min-width:0;' + TRUNCATE_PATH_CSS;
            instEl.textContent = s.instance;
            instEl.title = s.instance;
            // Linkify the instance row — for register/comb stages
            // the stage's flat odb_type/odb_id refers to the
            // instance.
            this._linkifyPin(instEl, s);
            instRow.appendChild(instEl);
            if (s.cell) {
                const cellEl = document.createElement('span');
                cellEl.textContent = s.cell;
                cellEl.title = s.cell;
                cellEl.style.cssText
                    = 'color:var(--fg-muted);flex-shrink:0;';
                instRow.appendChild(cellEl);
            }
            card.appendChild(instRow);
        }

        // ── Pin rows — shape depends on stage kind ─────────────────
        const pinTable = document.createElement('div');
        pinTable.style.cssText =
            'padding:4px 8px 6px;display:grid;' +
            'grid-template-columns:auto 1fr auto;column-gap:8px;row-gap:2px;';
        if (tint) pinTable.style.background = tint;

        // `clk` accepts either a single clock name (string) or an
        // array of clock names. When an array, each clock renders
        // as its OWN clickable chip — that's what powers the
        // per-clock fan-in expansion: the user clicks a clock chip
        // on a pin, and the widget back-walks the fan-in cone IN
        // THAT clock's domain. Single-string callers preserve the
        // original behaviour. Joined "/" labels (the previous
        // multi-clock display) are still produced when no pinRef
        // is available — without an ODB ref we can't trace fan-in
        // anyway, so the chip stays decorative.
        // `pinKind` controls how the fan-in trace is interpreted
        // when a chip on this pin is clicked. Defaults to 'data';
        // pass 'clock' for CK pin rows so the resulting expansion
        // labels its multi-clock convergence gate as a "clock-mux
        // convergence" instead of a (data-path) "domain mix".
        //
        // `pinName` is the FULL hierarchical path. For display we
        // strip the instance prefix (the stage card already shows
        // it in the header), but the full path stays in the
        // tooltip and is what gets threaded into the chip's
        // click closure for "from <pin>" convergence labels.
        const stageInst = s && s.instance || null;
        const addPinRow = (label, pinName, pinRef, clk, role,
                           pinKind, isOutputPin) => {
            const lab = document.createElement('span');
            lab.textContent = label;
            lab.style.cssText =
                'color:var(--fg-muted);font-weight:600;align-self:center;';
            const pathEl = document.createElement('span');
            // Show the leaf (D / Q / CK / A1 / ...) since the
            // instance is already in the card header — full
            // hierarchical paths in pin rows were redundant.
            // The tooltip keeps the full path so hovering still
            // surfaces the unique identifier.
            pathEl.textContent
                = pinName ? this._pinLeafName(pinName, stageInst) : '—';
            pathEl.title = pinName || '';
            pathEl.style.cssText =
                'color:var(--fg-primary);align-self:center;min-width:0;'
                + TRUNCATE_PATH_CSS;
            if (pinRef && pinRef.odb_type && pinRef.odb_id != null) {
                this._linkifyPin(pathEl, pinRef);
            }
            const clkContainer = document.createElement('span');
            clkContainer.style.cssText
                = 'align-self:center;display:inline-flex;'
                + 'align-items:center;gap:3px;flex-wrap:wrap;';
            // Normalise clk to an array; a falsy value renders the
            // dash. Filtering empties guards against backend nulls.
            const clkList = Array.isArray(clk)
                ? clk.filter(c => c)
                : (clk ? [clk] : []);
            if (clkList.length === 0) {
                const dash = document.createElement('span');
                dash.textContent = '—';
                dash.style.color = 'var(--fg-muted)';
                clkContainer.appendChild(dash);
            } else {
                // Chips render as inert badges. Clock-mix tracing
                // moved off per-chip clicks (which conflated
                // single-clock and mix scenarios and routinely
                // produced empty / stuck wrappers on phantom
                // propagation) and onto a single `↑ trace mix`
                // button at the end of the cluster — only present
                // when the row is multi-clock AND we have a pin
                // ref to walk back from.
                for (const c of clkList) {
                    const chip = document.createElement('span');
                    chip.textContent = c;
                    if (pinKind) {
                        chip.dataset.cdcPinKind = pinKind;
                    }
                    chip.style.cssText
                        = 'padding:0 6px;border-radius:8px;'
                        + 'font-size:11px;'
                        + 'background:var(--bg-header);'
                        + 'color:var(--fg-secondary);'
                        + 'white-space:nowrap;';
                    if (role === 'launch') {
                        chip.style.background = 'rgba(220, 64, 64, 0.18)';
                    } else if (role === 'capture') {
                        chip.style.background = 'rgba(76, 175, 80, 0.18)';
                    }
                    chip.title = role === 'launch'
                        ? `${c} — launches this transition (clock on `
                          + `the driving flop's CK pin)`
                        : (role === 'capture'
                            ? `${c} — captures this transition on `
                              + 'this flop\'s CK'
                            : `${c} — clock domain of this stage`);
                    clkContainer.appendChild(chip);
                }
                // Trace-mix button. Conditions:
                //   - row is multi-clock (≥ 2 clocks, otherwise
                //     there's no mix to trace);
                //   - pinRef resolves to an iterm/bterm so the
                //     backend can locate the start pin;
                //   - the pin is an INPUT-direction pin (OUT / Q
                //     output rows are skipped — by construction
                //     they redistribute their inputs' clocks, so
                //     a walk from OUT just re-derives what
                //     clicking the matching IN row would produce);
                //   - this card is not inside an expansion wrapper
                //     (no chained recursion from within an
                //     expansion — `disableFanInChips` carries the
                //     legacy name but means "no expand affordances
                //     in nested cards" and applies here too).
                const allowMixTrace = clkList.length >= 2
                    && !isOutputPin
                    && pinRef && pinRef.odb_type
                    && pinRef.odb_id != null
                    && !(opts && opts.disableFanInChips);
                if (allowMixTrace) {
                    const btn = document.createElement('a');
                    btn.textContent = '↑ trace mix';
                    btn.dataset.cdcTraceMix = '1';
                    if (pinKind) {
                        btn.dataset.cdcPinKind = pinKind;
                    }
                    btn.style.cssText
                        = 'cursor:pointer;color:var(--accent-tab);'
                        + 'text-decoration:underline;'
                        + 'text-decoration-style:dotted;'
                        + 'font-size:11px;margin-left:4px;'
                        + 'white-space:nowrap;';
                    btn.title
                        = `Trace where these ${clkList.length} clocks `
                        + 'merge upstream — finds the gate that mixes them.';
                    btn.addEventListener('click', () =>
                        this._toggleCdcClockMix(
                            pinRef, clkList.slice(), btn,
                            pinKind || 'data',
                            pinName));
                    clkContainer.appendChild(btn);
                }
            }
            pinTable.appendChild(lab);
            pinTable.appendChild(pathEl);
            pinTable.appendChild(clkContainer);
        };

        if (s.kind === 'comb') {
            // Combinational gate: IN (followed) + OUT, with optional
            // FAN-IN rows for the other inputs of multi-input cells.
            // Each input pin renders with its OWN clock-domain list so
            // a domain-mix gate's inputs show distinct badges (the
            // followed input matches the launch clock; aux pins on a
            // mix gate are in different domains).
            const launchClk = launch || null;
            const captureClk = capture || null;
            const inClks = Array.isArray(s.in_pin_clocks)
                ? s.in_pin_clocks : [];
            // Pass the clocks as an array so each renders as its own
            // chip (clickable per-clock for fan-in expansion). Falls
            // back to the stage's coarse `clock` when no per-input
            // clock array is on the backend payload.
            const inLabel = inClks.length ? inClks : (s.clock || null);
            const inRole = inClks.includes(launchClk)
                ? 'launch'
                : (inClks.includes(captureClk) ? 'capture' : 'domain');
            addPinRow('IN ', s.in_pin,
                { odb_type: s.in_pin_odb_type, odb_id: s.in_pin_odb_id },
                inLabel, inRole);
            // OUT carries the gate's domain — same as the followed
            // input on a same-domain gate; on a domain-mix gate it
            // depends on which clock dominates downstream (we tint
            // with the gate's stage clock = launch domain).
            addPinRow('OUT', s.out_pin,
                { odb_type: s.out_pin_odb_type, odb_id: s.out_pin_odb_id },
                s.clock || null, 'domain',
                /*pinKind=*/undefined, /*isOutputPin=*/true);
            const auxes = Array.isArray(s.aux_in_pins) ? s.aux_in_pins : [];
            for (const aux of auxes) {
                const auxClks = Array.isArray(aux.clocks) ? aux.clocks : [];
                // Same per-chip array policy as the IN row — each
                // clock on a multi-domain aux input becomes its own
                // clickable chip for fan-in tracing.
                const auxLabel = auxClks.length
                    ? auxClks : (s.clock || null);
                const auxRole = auxClks.includes(launchClk)
                    ? 'launch'
                    : (auxClks.includes(captureClk) ? 'capture' : 'domain');
                addPinRow('+IN', aux.name,
                    { odb_type: aux.odb_type, odb_id: aux.odb_id },
                    auxLabel, auxRole);
            }
        } else if (s.kind === 'port') {
            // Top-level INPUT port — only one pin matters and we expose
            // it as the OUT row (it drives the launch-side chain).
            addPinRow('OUT', s.out_pin || s.q_pin,
                { odb_type: s.out_pin_odb_type || s.q_pin_odb_type,
                  odb_id: s.out_pin_odb_id != null
                      ? s.out_pin_odb_id : s.q_pin_odb_id },
                s.clock || null, 'launch',
                /*pinKind=*/undefined, /*isOutputPin=*/true);
        } else {
            // Register stage — D + CK + Q rows. Launch flop's D is
            // in the launch domain (its own clock); crossover D is
            // also in the launch domain (the bug-relevant pin);
            // sync stages D is in the capture domain.
            const launchClk = s.launch_clock
                || (s.is_capture ? launch : null)
                || (s.is_launch ? (s.clock || launch) : null);
            const captureClk = s.capture_clock || s.clock || null;
            const dClk = s.is_capture
                ? launchClk
                : (s.is_launch ? (s.clock || launch) : captureClk);
            const qClk = s.is_launch ? (s.clock || launch) : captureClk;
            const dRole = s.is_capture
                ? 'launch'
                : (s.is_launch ? 'launch' : 'capture');
            // CK pin clock list comes straight from `clockDomains(CK)`
            // on the backend. Single-clock CK is the common case
            // (one chip shown, matches the role colour); multi-
            // clock CK (clock-mux'd flops) renders multiple chips
            // — this is the "why does the same FF appear as the
            // source for N clocks" UX cue. Falls back to the
            // capture-clock single string for older payloads
            // without `ck_pin_clocks`.
            const ckClocks = Array.isArray(s.ck_pin_clocks)
                ? s.ck_pin_clocks : (captureClk ? [captureClk] : []);
            addPinRow('D', s.d_pin,
                { odb_type: s.d_pin_odb_type, odb_id: s.d_pin_odb_id },
                dClk, dRole);
            addPinRow('CK', s.ck_pin || null,
                (s.ck_pin_odb_type && s.ck_pin_odb_id != null)
                    ? { odb_type: s.ck_pin_odb_type,
                        odb_id: s.ck_pin_odb_id }
                    : null,
                ckClocks, 'capture', 'clock');
            addPinRow('Q', s.q_pin,
                { odb_type: s.q_pin_odb_type, odb_id: s.q_pin_odb_id },
                qClk, s.is_launch ? 'launch' : 'capture',
                /*pinKind=*/undefined, /*isOutputPin=*/true);
            // Multi-clock-CK marker — built here so it has access
            // to `ckClocks`, but appended AFTER the pin table
            // below so it lands right under the CK row visually.
            if (ckClocks.length > 1) {
                const warn = document.createElement('div');
                warn.style.cssText
                    = 'padding:3px 8px;font-size:11px;'
                    + 'background:rgba(255, 167, 38, 0.15);'
                    + 'color:var(--fg-primary);'
                    + 'border-top:1px dashed rgba(255, 167, 38, 0.6);';
                warn.textContent = `⚠ multi-clock CK — `
                    + `${ckClocks.length} clocks reach this flop's `
                    + 'CK pin, so it is a launch source for all '
                    + 'of them. Common upstream causes: clock '
                    + 'mux / OR gate, or two `create_clock` '
                    + 'declarations on overlapping trees.';
                warn.title = ckClocks.join(', ')
                    + ' — every clock that propagates to this '
                    + 'flop\'s CK. Q (and everything reachable '
                    + 'from Q) inherits all of them.';
                // Stash on the card so the post-pinTable append
                // below picks it up. Keeps the if-block close to
                // the data that drives it.
                card._cdcMultiClockWarn = warn;
            }
        }
        card.appendChild(pinTable);
        if (card._cdcMultiClockWarn) {
            card.appendChild(card._cdcMultiClockWarn);
        }
        return card;
    }

    async _openCdcSettings() {
        let current = { instance_patterns: [], master_patterns: [] };
        try {
            await this._app.websocketManager.readyPromise;
            current = await this._requestWithTimeout({
                type: 'cdc_get_whitelist',
            });
        } catch (e) {
            console.warn('[CDC] get whitelist failed', e);
        }
        this._renderCdcSettingsModal(current);
    }

    _renderCdcSettingsModal(current) {
        // Inline modal overlay — keeps the UI single-pane (no popup
        // window) so it works in headless test runs too.
        const overlay = document.createElement('div');
        overlay.style.cssText =
            'position:absolute;inset:0;background:rgba(0,0,0,0.4);' +
            'display:flex;align-items:center;justify-content:center;z-index:10;';
        const modal = document.createElement('div');
        modal.style.cssText =
            'background:var(--bg-main);color:var(--fg-primary);' +
            'border:1px solid var(--border);border-radius:6px;' +
            'padding:16px 18px;min-width:480px;max-width:80%;' +
            'display:flex;flex-direction:column;gap:10px;font-size:12px;' +
            'box-shadow:0 4px 16px rgba(0,0,0,0.3);';

        const title = document.createElement('h3');
        title.textContent = 'CDC settings';
        title.style.cssText = 'margin:0;font-size:14px;';
        modal.appendChild(title);

        const explain = document.createElement('div');
        explain.style.cssText = 'color:var(--fg-muted);line-height:1.5;';
        explain.innerHTML =
            'One glob pattern per line. A path is reclassified as ' +
            '<b>synchronized (whitelisted)</b> when its capture flop matches ' +
            'either list. Lists are session-scoped — not persisted ' +
            'across openroad runs.';
        modal.appendChild(explain);

        const makeArea = (labelText, hint, initial) => {
            const block = document.createElement('div');
            block.style.cssText = 'display:flex;flex-direction:column;gap:4px;';
            const lbl = document.createElement('label');
            lbl.textContent = labelText;
            lbl.style.fontWeight = '600';
            block.appendChild(lbl);
            const ta = document.createElement('textarea');
            ta.rows = 5;
            ta.placeholder = hint;
            ta.value = (initial || []).join('\n');
            ta.style.cssText =
                'background:var(--bg-input);color:var(--fg-primary);' +
                'border:1px solid var(--border);border-radius:3px;' +
                'padding:6px;font-family:monospace;font-size:12px;resize:vertical;';
            block.appendChild(ta);
            return { block, ta };
        };
        const inst = makeArea(
            'Instance whitelist',
            'e.g. u_top/u_sync_a/q_reg or  *u_sync*',
            (current && current.instance_patterns) || []);
        const master = makeArea(
            'Master (cell-name) whitelist',
            'e.g. SYNC_FF_X1 or  *SYNC*',
            (current && current.master_patterns) || []);
        modal.appendChild(inst.block);
        modal.appendChild(master.block);

        const btnRow = document.createElement('div');
        btnRow.style.cssText =
            'display:flex;justify-content:flex-end;gap:6px;margin-top:6px;';
        const cancel = document.createElement('button');
        cancel.textContent = 'Cancel';
        cancel.style.cssText =
            'padding:4px 12px;background:var(--bg-input);' +
            'color:var(--fg-primary);border:1px solid var(--border);' +
            'border-radius:3px;cursor:pointer;';
        cancel.addEventListener('click', () => overlay.remove());
        btnRow.appendChild(cancel);

        const apply = document.createElement('button');
        apply.textContent = 'Apply & re-scan';
        apply.style.cssText =
            'padding:4px 12px;background:var(--accent-tab);color:#fff;' +
            'border:none;border-radius:3px;cursor:pointer;';
        apply.addEventListener('click', async () => {
            const splitClean = (txt) =>
                txt.split('\n')
                   .map(l => l.trim())
                   .filter(l => l.length > 0)
                   .join(',');
            try {
                await this._requestWithTimeout({
                    type: 'cdc_set_whitelist',
                    instance_patterns: splitClean(inst.ta.value),
                    master_patterns:   splitClean(master.ta.value),
                });
                overlay.remove();
                // Re-scan so the matrix reflects the new classification.
                this._loadCdcOverview();
            } catch (e) {
                console.warn('[CDC] set whitelist failed', e);
            }
        });
        btnRow.appendChild(apply);
        modal.appendChild(btnRow);

        overlay.appendChild(modal);
        // Anchor to the widget so positioning is contained, not page-wide.
        if (this.element) {
            this.element.style.position = 'relative';
            this.element.appendChild(overlay);
        } else {
            document.body.appendChild(overlay);
        }
    }

// ── Helpers ───────────────────────────────────────────────────────────────

    // Build a row of toolbar filter buttons. Replaces four hand-rolled
    // copies (Clocks, Port Delays, Exceptions, Endpoint Kind) — each had
    // its own active/inactive style toggle and click handler. The
    // dataset prefix preserves each tab's existing attribute names so
    // tests and count-update helpers keep working.
    //
    // Args:
    //   container     — parent element to append buttons to
    //   defs          — [{ key, label, title?, ...extras }]; extras are
    //                   stored as dataset[`${prefix}${ExtraKey}`] when
    //                   included. `title` is attached as the button's
    //                   tooltip and is the recommended way to explain
    //                   what each filter narrows to.
    //   initialKey    — which button is rendered active on first paint
    //   fontSize      — '11px' or '12px' (the only two sizes in use)
    //   datasetPrefix — e.g. 'clkFilter', 'kind', 'excFilter', 'pd'
    //   extras        — list of extra dataset keys to thread from defs
    //   onSelect      — (key) => void, called after the active toggle
    // Returns the { key → button } map.
    _makeFilterButtons({ container, defs, initialKey, fontSize = '11px',
                         datasetPrefix, extras = [], onSelect }) {
        const baseStyle = (active) =>
            `padding:2px 8px;font-size:${fontSize};cursor:pointer;border-radius:3px;` +
            (active
                ? 'background:var(--accent-tab);color:#fff;'
                  + 'border:1px solid var(--accent-tab);'
                : 'background:var(--bg-input);color:var(--fg-secondary);'
                  + 'border:1px solid var(--border);');
        const btns = {};
        for (const def of defs) {
            const btn = document.createElement('button');
            btn.dataset[`${datasetPrefix}Key`]   = def.key;
            btn.dataset[`${datasetPrefix}Label`] = def.label;
            for (const extraKey of extras) {
                if (def[extraKey] != null) {
                    btn.dataset[`${datasetPrefix}${extraKey
                        .charAt(0).toUpperCase() + extraKey.slice(1)}`]
                        = String(def[extraKey]);
                }
            }
            btn.textContent = def.label;
            if (def.title) btn.title = def.title;
            btn.style.cssText = baseStyle(def.key === initialKey);
            btn.addEventListener('click', () => {
                for (const [k, b] of Object.entries(btns)) {
                    b.style.cssText = baseStyle(k === def.key);
                }
                onSelect(def.key);
            });
            btns[def.key] = btn;
            container.appendChild(btn);
        }
        return btns;
    }

    // Build a multi-select clock-domain filter — a "Clock: N selected ▾"
    // trigger button + popup panel of monospace checkboxes, one per
    // clock with its endpoint count. Used by both the Endpoints and
    // Port Delays tabs, which had nearly-identical hand-rolled copies
    // before this helper landed.
    //
    // Args:
    //   container — toolbar element to append into
    //   onChange  — (selected: null | Set<string>) => void
    //               null  → "all clocks" sentinel
    //               Set   → explicit subset (possibly empty)
    //   title     — tooltip text shown on the trigger + label
    //   labelText — toolbar label, default 'Clock:'
    //
    // Returns a controller with:
    //   wrap, sep, lbl, trigger, panel — the DOM nodes (so callers can
    //                                    style/show/hide as needed)
    //   populate(clocksTotal, opts)   — rebuild the panel from a clock
    //                                    name → count map. Pass
    //                                    opts.noneCount > 0 to add a
    //                                    "(no clock)" row at the end
    //                                    (Port Delays uses this for
    //                                    exception-only entries).
    //   getSelected()                 — current selection (null | Set)
    //   getValue()                    — backend-ready string: 'all',
    //                                    or comma-separated list (the
    //                                    sentinel '__none__' is sent
    //                                    verbatim for "(no clock)")
    //   matches(clockName)            — predicate for client-side
    //                                    filtering; pass null/undefined
    //                                    for entries with no clock
    _makeClockCheckboxFilter({ container, onChange, title, labelText }) {
        // ── DOM scaffolding ──────────────────────────────────────────
        const sep = document.createElement('span');
        sep.style.cssText =
            'width:1px;height:14px;background:var(--border);margin:0 4px;';
        sep.style.display = 'none';
        container.appendChild(sep);

        const lbl = document.createElement('span');
        lbl.textContent = labelText || 'Clock:';
        lbl.style.cssText =
            'font-size:12px;color:var(--fg-muted);font-weight:600;' +
            'flex-shrink:0;';
        if (title) lbl.title = title;
        lbl.style.display = 'none';
        container.appendChild(lbl);

        // position:relative wrapper anchors the absolute popup within
        // the toolbar's coordinate space.
        const wrap = document.createElement('span');
        wrap.style.cssText = 'position:relative;display:inline-block;';
        wrap.style.display = 'none';
        container.appendChild(wrap);

        const trigger = document.createElement('button');
        trigger.type = 'button';
        trigger.className = 'sdc-clock-filter-trigger';
        trigger.dataset.clockFilterTrigger = 'true';
        trigger.style.cssText =
            'padding:1px 6px;font-size:12px;font-family:monospace;' +
            'cursor:pointer;border:1px solid var(--border);border-radius:3px;' +
            'background:var(--bg-input);color:var(--fg-primary);';
        if (title) trigger.title = title;
        trigger.textContent = 'All clocks ▾';
        wrap.appendChild(trigger);

        const panel = document.createElement('div');
        panel.className = 'sdc-clock-filter-panel';
        panel.dataset.clockFilterPanel = 'true';
        panel.style.cssText =
            'display:none;position:absolute;top:100%;left:0;margin-top:2px;' +
            'min-width:200px;max-height:280px;overflow-y:auto;' +
            'background:var(--bg-input);color:var(--fg-primary);' +
            'border:1px solid var(--border);border-radius:3px;' +
            'box-shadow:0 2px 8px rgba(0,0,0,0.3);z-index:50;' +
            'padding:4px 0;font-size:12px;';
        wrap.appendChild(panel);

        // ── State + behavior ─────────────────────────────────────────
        // null = "all clocks" sentinel; Set = explicit subset (which
        // may be empty for the "no clocks selected" state).
        let selected = null;
        let names = [];
        let totals = {};
        let hasNoneOption = false;
        const NONE_KEY = '__none__';

        let outsideHandler = null;
        const togglePanel = (open) => {
            const wantOpen = open === undefined
                ? panel.style.display === 'none' : !!open;
            panel.style.display = wantOpen ? 'block' : 'none';
            if (wantOpen) {
                outsideHandler = (ev) => {
                    if (!panel.contains(ev.target)
                        && !trigger.contains(ev.target)) togglePanel(false);
                };
                document.addEventListener('mousedown', outsideHandler);
            } else if (outsideHandler) {
                document.removeEventListener('mousedown', outsideHandler);
                outsideHandler = null;
            }
        };
        trigger.addEventListener('click', (e) => {
            e.stopPropagation();
            togglePanel();
        });

        const refreshLabel = () => {
            const total = names.reduce((s, n) => s + (totals[n] || 0), 0)
                + (hasNoneOption ? (totals[NONE_KEY] || 0) : 0);
            const totalKeys = names.length + (hasNoneOption ? 1 : 0);
            const renderName = (k) => k === NONE_KEY ? '(no clock)' : k;
            let label;
            if (selected === null) {
                label = `All clocks (${total})`;
            } else if (selected.size === 0) {
                label = 'No clocks selected';
            } else if (selected.size === 1) {
                const only = [...selected][0];
                label = `${renderName(only)} (${totals[only] || 0})`;
            } else {
                label = `${selected.size} of ${totalKeys} clocks`;
            }
            trigger.textContent = `${label} ▾`;
        };

        const populate = (clocksTotal, opts = {}) => {
            const map = clocksTotal || {};
            // Sort the regular clock names alphabetically; pull the
            // "(no clock)" sentinel to the bottom of the list when
            // present so it's visually distinct.
            const allKeys = Object.keys(map);
            const regularNames = allKeys.filter(k => k !== NONE_KEY).sort();
            const noneCountFromOpts = opts.noneCount;
            const noneCount = noneCountFromOpts != null
                ? noneCountFromOpts
                : (map[NONE_KEY] || 0);
            names = regularNames;
            totals = { ...map };
            if (noneCount > 0) {
                totals[NONE_KEY] = noneCount;
                hasNoneOption = true;
            } else {
                delete totals[NONE_KEY];
                hasNoneOption = false;
            }

            // Reconcile prior selection against the new key set: drop
            // names that have vanished. If the user previously chose a
            // non-empty subset and ALL of those keys disappeared,
            // collapse to "all" so the UI doesn't silently filter
            // against missing clocks. An originally-empty selection
            // (the deliberate "show none" state) survives untouched.
            if (selected instanceof Set) {
                const keys = new Set(names);
                if (hasNoneOption) keys.add(NONE_KEY);
                const had = selected.size;
                const survivors = new Set();
                for (const k of selected) {
                    if (keys.has(k)) survivors.add(k);
                }
                selected = (had > 0 && survivors.size === 0)
                    ? null : survivors;
            }

            // Rebuild panel.
            panel.innerHTML = '';
            const rowStyle =
                'display:flex;align-items:center;gap:6px;padding:3px 8px;' +
                'cursor:pointer;user-select:none;font-family:monospace;';
            const buildRow = (key, displayName, count) => {
                const isChecked = selected === null ? true : selected.has(key);
                const row = document.createElement('label');
                row.style.cssText = rowStyle;
                row.dataset.clockFilterOption = key;
                const cb = document.createElement('input');
                cb.type = 'checkbox';
                cb.checked = isChecked;
                cb.style.cssText = 'flex-shrink:0;';
                cb.addEventListener('change', () => {
                    // First per-key toggle materialises the selection
                    // set: start from "everything" (the current null
                    // sentinel) and mutate from there.
                    if (selected === null) {
                        selected = new Set(names);
                        if (hasNoneOption) selected.add(NONE_KEY);
                    }
                    if (cb.checked) selected.add(key);
                    else            selected.delete(key);
                    // Re-checking every key collapses back to the
                    // "all" sentinel (cheaper backend path + cleaner
                    // trigger label).
                    const fullSize = names.length + (hasNoneOption ? 1 : 0);
                    if (selected.size === fullSize) selected = null;
                    refreshLabel();
                    onChange && onChange(selected);
                });
                row.appendChild(cb);
                const lblSpan = document.createElement('span');
                lblSpan.style.cssText = 'flex:1;';
                lblSpan.textContent = displayName;
                row.appendChild(lblSpan);
                const cntSpan = document.createElement('span');
                cntSpan.style.cssText =
                    'color:var(--fg-muted);font-size:11px;';
                cntSpan.textContent = `(${count})`;
                row.appendChild(cntSpan);
                row.addEventListener('mouseenter',
                    () => { row.style.background = 'var(--bg-hover)'; });
                row.addEventListener('mouseleave',
                    () => { row.style.background = ''; });
                panel.appendChild(row);
            };

            for (const n of names) buildRow(n, n, map[n] || 0);
            if (hasNoneOption) {
                buildRow(NONE_KEY, '(no clock)', noneCount);
            }

            const totalKeys = names.length + (hasNoneOption ? 1 : 0);
            const visible = totalKeys >= 2;
            sep.style.display     = visible ? 'inline-block' : 'none';
            lbl.style.display     = visible ? '' : 'none';
            wrap.style.display    = visible ? 'inline-block' : 'none';
            if (!visible) togglePanel(false);
            refreshLabel();
        };

        const getValue = () => {
            if (selected === null) return 'all';
            return [...selected].join(',');
        };
        const matches = (clockName) => {
            if (selected === null) return true;
            const key = clockName == null ? NONE_KEY : clockName;
            return selected.has(key);
        };

        return {
            wrap, sep, lbl, trigger, panel,
            populate,
            getSelected: () => selected,
            getValue,
            matches,
        };
    }

    // Convert a clock period (in the current display time unit) to a frequency string.
    _periodToFreq(period) {
        // Generated clocks whose period hasn't been resolved come through
        // with period == 0; return a clear marker instead of "Infinity GHz".
        if (!Number.isFinite(period) || period <= 0) return '—';
        const scale = { ps: 1e-12, ns: 1e-9, us: 1e-6, ms: 1e-3, s: 1 }[this._timeUnit] || 1e-9;
        const hz = 1 / (period * scale);
        if (hz >= 1e9) return `${(hz / 1e9).toPrecision(4)} GHz`;
        if (hz >= 1e6) return `${(hz / 1e6).toPrecision(4)} MHz`;
        if (hz >= 1e3) return `${(hz / 1e3).toPrecision(4)} kHz`;
        return `${hz.toPrecision(4)} Hz`;
    }

    // ── Data loading ─────────────────────────────────────────────────────────

    // Wraps a request promise with a 10-second timeout. The timer is
    // cancelled when whichever side wins the race so we don't leave a
    // long-lived setTimeout around for every fast request.
    // Wrap a websocket request with a soft timeout. The default of 60s
    // is generous enough for everything-except-the-most-pathological
    // SDC walks; expensive ones (sdc_endpoint_list on a million-instance
    // design, sdc_resolve_gen_clocks with hundreds of generated clocks)
    // can opt into longer caps via the second argument.
    //
    // On timeout the returned promise rejects with an Error tagged
    // `isTimeout = true`. Callers should branch on that to render the
    // "Still loading — Check again" banner via _showLoadError, which
    // re-issues the same fetch (the server will return cached results
    // quickly if the original walk has since finished).
    //
    // The original websocket request is NOT cancelled — leaving it in
    // pending lets a slow-but-eventually-arriving response still
    // resolve cleanly to a no-op (we already gave up on the promise,
    // but the websocket-manager dispatch table absorbs orphaned ids).
    _requestWithTimeout(msg, timeoutMs) {
        const ms = (typeof timeoutMs === 'number' && timeoutMs > 0)
            ? timeoutMs : 60000;
        const req = this._app.websocketManager.request(msg);
        let timeoutId;
        const timeout = new Promise((_, reject) => {
            timeoutId = setTimeout(() => {
                const sec = Math.round(ms / 1000);
                const err = new Error(
                    `sdc request '${msg.type}' timed out after ${sec}s`);
                err.isTimeout = true;
                reject(err);
            }, ms);
        });
        return Promise.race([req, timeout]).finally(() => clearTimeout(timeoutId));
    }

    // Render a load failure inside `target`, with an optional retry
    // affordance. The "Check again" button is the key UX for slow
    // operations: when a request times out, the work is usually still
    // running on the server (or already finished and cached), so a
    // retry typically resolves quickly. Used by every tab's catch
    // handler so the error UX is uniform.
    //
    // Args:
    //   target  — DOM element whose innerHTML is replaced with the message
    //   opName  — short label of what failed ("clocks", "port delays", …)
    //   error   — caught Error / string
    //   retryFn — optional () => void, called when the user clicks retry.
    //             Pass `null` for non-retryable errors.
    _showLoadError(target, opName, error, retryFn) {
        if (!target) return;
        target.innerHTML = '';
        const msg = (error && error.message) || String(error);
        const isTimeout = !!(error && error.isTimeout)
            || /timed out/i.test(msg);
        const wrap = document.createElement('div');
        wrap.style.cssText =
            'padding:12px;color:var(--fg-muted);font-style:italic;' +
            'display:flex;align-items:center;gap:8px;flex-wrap:wrap;';
        const text = document.createElement('span');
        text.textContent = isTimeout
            ? `Still loading ${opName}… the server may need more time. ` +
              'The operation is likely still running and will be cached on ' +
              'the next try.'
            : `Error loading ${opName}: ${msg}`;
        wrap.appendChild(text);
        if (retryFn) {
            const btn = document.createElement('button');
            btn.textContent = isTimeout ? 'Check again' : 'Retry';
            btn.style.cssText =
                'padding:3px 10px;font-size:12px;cursor:pointer;' +
                'background:var(--bg-input);color:var(--fg-primary);' +
                'border:1px solid var(--border);border-radius:3px;';
            btn.addEventListener('click', () => retryFn());
            wrap.appendChild(btn);
        }
        target.appendChild(wrap);
    }

    // Inspect an ODB object (pin/port/instance) that was referenced by the
    // SDC response.  The backend emits {odb_type, odb_id} pairs next to each
    // linkable name, so we can ask the existing `inspect` handler to resolve
    // them directly — no separate server-side name lookup.
    //
    // Fails silently when the app doesn't expose the inspector hooks (e.g.
    // in unit tests without a full GoldenLayout shell).
    async _inspectByOdb(odbType, odbId) {
        if (!odbType || odbId == null || odbId < 0) return;
        try {
            await this._app.websocketManager.readyPromise;
            // Dedicated request type — backend's SelectHandler
            // ::handleInspectByOdb resolves directly from ODB without
            // routing through the canvas-pick selectables[] list.
            const data = await this._requestWithTimeout({
                type: 'inspect_by_odb',
                odb_type: odbType,
                odb_id:   odbId,
            });
            if (this._app.updateInspector) this._app.updateInspector(data);
            if (this._app.focusComponent) this._app.focusComponent('Inspector');
        } catch (e) {
            console.warn(`[SDC] inspect ${odbType}:${odbId} failed`, e);
        }
    }

    // Decorate an element so clicking it inspects the referenced ODB object.
    // Two call styles are accepted, matching the two JSON shapes the backend
    // uses for ODB refs:
    //   this._linkifyPin(el, { odb_type, odb_id })   — direct ref
    //   this._linkifyPin(el, host, 'prefix')          — reads
    //       host.<prefix>_odb_type / host.<prefix>_odb_id  (flat-field style)
    // Either form is a no-op when the odb fields are missing, so it's safe
    // to call on names that didn't resolve to a concrete ODB object server-side.
    _linkifyPin(el, refOrHost, prefix) {
        if (!el || !refOrHost) return;
        const ref = prefix
            ? { odb_type: refOrHost[`${prefix}_odb_type`],
                odb_id:   refOrHost[`${prefix}_odb_id`] }
            : refOrHost;
        if (!ref.odb_type || ref.odb_id == null) return;
        el.style.cursor = 'pointer';
        el.style.textDecoration = 'underline';
        el.style.textDecorationStyle = 'dotted';
        el.style.textDecorationColor = 'var(--fg-muted)';
        el.addEventListener('click', (e) => {
            e.stopPropagation();
            this._inspectByOdb(ref.odb_type, ref.odb_id);
        });
    }

    // Wire an element to display a hierarchical path with left-side
    // truncation: long names lose the front (highest hierarchy) of
    // the path so the leaf — pin / instance / net at the bottom of
    // the hierarchy — stays visible. Always sets `title` so the user
    // can hover for the full text whether or not it actually clipped.
    //
    // `extraCss` is appended after TRUNCATE_PATH_CSS so caller-specific
    // layout rules (font-family, color, flex sizing) win.
    _setTruncatedPath(el, fullText, extraCss) {
        if (!el) return;
        el.textContent = fullText == null ? '' : String(fullText);
        el.title = el.textContent;
        el.style.cssText = TRUNCATE_PATH_CSS + (extraCss || '');
    }

    // Strip the instance-path prefix from a pin path so the leaf
    // ("D" / "Q" / "CK" / "RN" / …) shows alone. The Endpoints tab's
    // instance card already names the instance in its header, so
    // every pin row inside that card showing the full hierarchical
    // path was redundant and forced unnecessary truncation. Falls
    // back to "everything after the last `/`" when the instance
    // prefix doesn't match (top-level ports, bus expansion, etc).
    _pinLeafName(pinName, instancePath) {
        if (!pinName) return '';
        if (instancePath && pinName.startsWith(instancePath + '/')) {
            return pinName.slice(instancePath.length + 1);
        }
        const slash = pinName.lastIndexOf('/');
        return slash >= 0 ? pinName.slice(slash + 1) : pinName;
    }

    // Append a small italic comment row to the given parent element when the
    // SDC object carries a `-comment "..."`. Used on clock cards, exception
    // rows, and clock-group cards (every SdcCmdComment subclass).
    _appendComment(parent, comment) {
        if (!comment) return;
        const note = document.createElement('div');
        note.style.cssText =
            'padding:3px 10px;font-size:12px;font-style:italic;' +
            'color:var(--fg-muted);border-top:1px solid var(--border-subtle);' +
            'white-space:pre-wrap;word-break:break-word;';
        // Visual cue that this is a user comment, not a generated annotation.
        note.textContent = `“${comment}”`;
        parent.appendChild(note);
    }

    async _loadData() {
        if (this._loaded || this._loading) return;
        this._loading = true;
        this._showTreePlaceholder('Loading…');
        try {
            console.log('[SDC] waiting for WebSocket…');
            await this._app.websocketManager.readyPromise;
            console.log('[SDC] WebSocket ready, sending sdc_clocks + sdc_clock_modes');

            const [clocksResp, modesResp] = await Promise.all([
                this._requestWithTimeout({ type: 'sdc_clocks' }),
                this._requestWithTimeout({ type: 'sdc_clock_modes' }),
            ]);
            console.log('[SDC] responses received:', clocksResp, modesResp);

            this._clocks = clocksResp.clocks || [];
            this._timeUnit = clocksResp.time_unit || 'ns';
            this._clockTree = clocksResp.clock_tree || [];
            this._clockMap = {};
            for (const clk of this._clocks) {
                this._clockMap[clk.name] = clk;
            }

            // Merge case_analysis (set_case_analysis) and logic_values
            // (set_logic_zero/one/dc) — both pin a design node to a constant
            // and are rendered in the same strip, tagged so the user can tell
            // them apart.
            const case_vals = (modesResp.case_analysis || [])
                .map(e => ({ ...e, kind: 'case' }));
            const logic_vals = (modesResp.logic_values || [])
                .map(e => ({ ...e, kind: 'logic' }));
            this._caseAnalysis = [...case_vals, ...logic_vals];
            this._currentMode = modesResp.current_mode || '';

            this._loaded = true;
            this._renderClockCards();
            this._renderCaseStrip();
            // The Clock Groups matrix is keyed on `this._clocks`, so re-run
            // it whenever fresh clock data arrives — covers the case where
            // the user opened that tab before sdc_clocks finished loading.
            if (this._cgLoaded && this._cgData) {
                this._renderClockGroups(this._cgData);
            }
        } catch (e) {
            console.error('[SDC] load error:', e);
            this._showLoadError(this._cardScrollArea, 'clocks', e, () => {
                this._loaded = false;
                this._loading = false;
                this._loadData();
            });
        } finally {
            this._loading = false;
        }
    }

    // ── Clock card rendering ─────────────────────────────────────────────────

    // Walk the clock tree depth-first and render one card per clock.
    // Append running counts to the master/generated/all clock-filter
    // buttons. Re-rendered every time the clock list changes so each
    // bucket reflects the current Sdc state.
    _updateClockFilterCounts() {
        if (!this._clkFilterBtns) return;
        const list = this._clocks || [];
        const all = list.length;
        const gen = list.filter(c => c && c.is_generated).length;
        const virt = list.filter(c => c && c.is_virtual).length;
        // Master = primary, real-source clocks (not generated, not virtual).
        const master = list.filter(
            c => c && !c.is_generated && !c.is_virtual).length;
        const counts = { all, master, generated: gen, virtual: virt };
        for (const [key, btn] of Object.entries(this._clkFilterBtns)) {
            const lbl = btn.dataset.clkFilterLabel || btn.textContent;
            const n = counts[key];
            btn.textContent = (typeof n === 'number') ? `${lbl} (${n})` : lbl;
        }
    }

    // Per-clock endpoint chip — populated lazily by the (list) link.
    // `this._clockEndpointCounts` is null while uncomputed, an object
    // (`{name: count, ...}`) once the user has triggered the count walk.
    // `this._loadingClockEndpointCounts` guards against a double-fire
    // when the user clicks (list) on multiple cards in quick succession.
    _renderEndpointChipContent(chip, clockName) {
        chip.innerHTML = '';
        const counts = this._clockEndpointCounts;
        if (this._loadingClockEndpointCounts) {
            chip.textContent = 'endpoints: …';
            chip.title = 'counting endpoints — building timing graph';
            return;
        }
        if (counts && Object.prototype.hasOwnProperty.call(counts, clockName)) {
            chip.textContent = `endpoints: ${counts[clockName]}`;
            chip.title = 'number of timing-endpoint pins reached by this ' +
                'clock domain (per-pin count, no instance dedup)';
            return;
        }
        if (counts) {
            // Counts have been computed but this clock had zero endpoints.
            chip.textContent = 'endpoints: 0';
            chip.title = 'no timing-endpoint pin reached by this clock domain';
            return;
        }
        // Uncomputed — show a (list) link the user can press to trigger
        // the walk. Building the timing graph + endpoint cache is the
        // slowest SDC call on big designs, so it stays opt-in.
        const lbl = document.createTextNode('endpoints: ? ');
        chip.appendChild(lbl);
        const link = document.createElement('a');
        link.href = '#';
        link.textContent = '(list)';
        link.style.cssText =
            'color:var(--accent-tab);text-decoration:underline;cursor:pointer;';
        link.title = 'count endpoints reached by every clock — first call ' +
            'builds the timing graph and may take a moment on large designs';
        link.addEventListener('click', (e) => {
            e.preventDefault();
            this._loadClockEndpointCounts();
        });
        chip.appendChild(link);
    }

    _refreshAllEndpointChips() {
        if (!this._cardScrollArea) return;
        const chips = this._cardScrollArea.querySelectorAll(
            '.sdc-clock-card [data-endpoints-chip]');
        for (const chip of chips) {
            const card = chip.closest('.sdc-clock-card');
            const name = card && card.dataset && card.dataset.clockName;
            if (name) {
                this._renderEndpointChipContent(chip, name);
            }
        }
    }

    async _loadClockEndpointCounts() {
        if (this._loadingClockEndpointCounts) return;
        this._loadingClockEndpointCounts = true;
        this._refreshAllEndpointChips();
        try {
            await this._app.websocketManager.readyPromise;
            // Same compute cost ceiling as sdc_endpoint_list — first call
            // builds the timing graph on a fresh design. Allow up to
            // 5 minutes for huge cases; subsequent calls hit the cache.
            const data = await this._requestWithTimeout(
                { type: 'sdc_endpoint_counts' },
                /*timeoutMs=*/300000);
            this._clockEndpointCounts = data.clocks_total || {};
        } catch (e) {
            console.error('[SDC] endpoint count error:', e);
            // Reset cache so the (list) link comes back, letting the
            // user retry instead of being stuck on a stale "…".
            this._clockEndpointCounts = null;
        } finally {
            this._loadingClockEndpointCounts = false;
            this._refreshAllEndpointChips();
        }
    }

    _renderClockCards() {
        this._cardScrollArea.innerHTML = '';
        // Refresh per-bucket counts on the master/generated/all toolbar
        // buttons so the user can see at a glance how many clocks each
        // filter would surface (matches the counts pattern on the
        // Endpoints kind toolbar and Port Delays direction toolbar).
        this._updateClockFilterCounts();

        // Show / hide the "Resolve generated" button based on whether any
        // generated clocks still have an unresolved period. We do this on
        // every render — including after a successful resolve — so the
        // button disappears once everything is computed (and the explainer
        // status text disappears with it on the next render).
        const unresolvedCount = (this._clocks || []).filter(
            c => c && c.is_generated
                 && (!Number.isFinite(c.period) || c.period <= 0)).length;
        if (this._resolveBtn) {
            const show = unresolvedCount > 0;
            this._resolveBtn.style.display = show ? '' : 'none';
            if (this._resolveStatus) {
                // Keep status visible as long as the button is, otherwise
                // clear it so a stale "resolved N" doesn't linger.
                if (!show) this._resolveStatus.textContent = '';
                this._resolveStatus.style.display = show ? '' : 'none';
            }
        }

        if (this._clockTree.length === 0) {
            this._showTreePlaceholder('No clocks defined in SDC.');
            return;
        }
        const STATS_W   = 160;
        const cardW     = Math.max(480, this._cardScrollArea.clientWidth - 16);
        const WAVE_W    = cardW - STATS_W;
        const WAVE_H    = 70;   // waveform area + axis line + edge-time labels
        const ROW_H     = 46;   // waveform drawing height passed to _drawWaveform
        const DRAW_W    = WAVE_W - 2;  // 2px right margin prevents stroke clipping
        const AX_Y      = ROW_H + 6;
        const LABEL_Y   = AX_Y + 10;
        const clkFilter = this._clkFilter || 'all';

        // time → x within the SVG
        const tx = (t, dur) => (t / dur) * DRAW_W;

        // Tree-view rendering: instead of flat "margin-left: depth * 16",
        // each parent owns a child container that's indented + carries a
        // vertical guide line back to the parent.  Makes parent/child
        // relationships visually unambiguous, especially for designs with
        // many generated clocks.  In master/generated filter modes we still
        // flatten — the parent of a generated clock isn't visible there, so
        // a tree would have orphans.
        const useTree = clkFilter === 'all';

        const visitNode = (node, parentContainer) => {
            const clk = this._clockMap[node.name];

            // Always recurse into children before the visibility check so that
            // generated clocks appear when the "Generated" filter is active.
            const recurseInto = (container) => {
                if (node.children)
                    for (const c of node.children) visitNode(c, container);
            };

            if (!clk) { recurseInto(parentContainer); return; }

            const shouldShow =
                clkFilter === 'all' ||
                (clkFilter === 'master'    && !clk.is_generated && !clk.is_virtual) ||
                (clkFilter === 'generated' &&  clk.is_generated) ||
                (clkFilter === 'virtual'   &&  clk.is_virtual);

            if (!shouldShow) { recurseInto(parentContainer); return; }

            // ── Card shell ───────────────────────────────────────────────
            // Indentation happens via the parent's children-container in tree
            // mode; in flatten mode the card sits directly in the scroll area
            // with no left margin. Tagging with a class so DOM selectors can
            // count clock cards uniformly across tree and flat layouts.
            const card = document.createElement('div');
            card.className = 'sdc-clock-card';
            card.dataset.clockName = clk.name;
            card.style.cssText =
                'margin-bottom:8px;' +
                'border:1px solid var(--border);border-radius:4px;overflow:hidden;';

            // Header: name + type badge (master / generated / virtual)
            // + optional ratio badge (÷N / ×N / inv) for generated clocks.
            const hdr = document.createElement('div');
            hdr.style.cssText =
                'display:flex;align-items:center;gap:8px;padding:4px 8px;' +
                'background:var(--bg-header);border-bottom:1px solid var(--border);';
            const nameEl = document.createElement('span');
            nameEl.style.cssText =
                'font-family:monospace;font-size:12px;font-weight:600;' +
                'color:var(--fg-primary);flex:1;' + TRUNCATE_PATH_CSS;
            nameEl.textContent = clk.name;
            nameEl.title = clk.name;
            hdr.appendChild(nameEl);

            const typeInfo = clk.is_virtual
                ? { label: 'virtual',
                    fg: 'var(--sdc-virtual-fg)',   bg: 'var(--sdc-virtual-bg)' }
                : clk.is_generated
                ? { label: 'generated',
                    fg: 'var(--sdc-generated-fg)', bg: 'var(--sdc-generated-bg)' }
                : { label: 'master',
                    fg: 'var(--sdc-master-fg)',    bg: 'var(--sdc-master-bg)' };
            const typeBadge = document.createElement('span');
            typeBadge.style.cssText =
                'font-size:11px;padding:1px 6px;border-radius:3px;font-weight:600;' +
                `background:${typeInfo.bg};color:${typeInfo.fg};`;
            typeBadge.textContent = typeInfo.label;
            typeBadge.title = clk.is_virtual
                ? 'virtual clock — no source pin (create_clock without ' +
                  '[get_ports …]). Used as a reference for I/O delays.'
                : clk.is_generated
                ? 'generated clock — derived from a master clock by a ' +
                  'create_generated_clock command (divide / multiply / ' +
                  'invert / -edges).'
                : 'primary (master) clock — defined by create_clock on a ' +
                  'design pin.';
            hdr.appendChild(typeBadge);

            if (clk.is_generated) {
                const ratioKind =
                    clk.divide_by > 1   ? 'divide_by' :
                    clk.multiply_by > 1 ? 'multiply_by' :
                    clk.invert          ? 'invert' : null;
                const ratio =
                    ratioKind === 'divide_by'   ? `÷${clk.divide_by}` :
                    ratioKind === 'multiply_by' ? `×${clk.multiply_by}` :
                    ratioKind === 'invert'      ? 'inv' : null;
                if (ratio) {
                    const ratioBadge = document.createElement('span');
                    ratioBadge.style.cssText =
                        'font-size:11px;padding:1px 5px;border-radius:3px;' +
                        'background:var(--bg-input);color:var(--accent-tab);font-weight:600;';
                    ratioBadge.dataset.clockBadge = ratioKind;
                    if (ratioKind === 'divide_by') {
                        ratioBadge.dataset.value = String(clk.divide_by);
                    } else if (ratioKind === 'multiply_by') {
                        ratioBadge.dataset.value = String(clk.multiply_by);
                    }
                    ratioBadge.textContent = ratio;
                    ratioBadge.title =
                        ratioKind === 'divide_by'
                            ? `create_generated_clock -divide_by ${clk.divide_by}` +
                              ` — this clock's period is ${clk.divide_by}× the master's`
                        : ratioKind === 'multiply_by'
                            ? `create_generated_clock -multiply_by ${clk.multiply_by}` +
                              ` — this clock's frequency is ${clk.multiply_by}×` +
                              ` the master's (period divided)`
                        : `create_generated_clock -invert — this clock is the ` +
                          `master's waveform with rise/fall edges swapped`;
                    hdr.appendChild(ratioBadge);
                }
            }

            // Propagation badge: `set_propagated_clock` switches a clock from
            // the ideal-zero-delay model to using computed network delays.
            // Show it as a small secondary badge so users can tell at a glance
            // which clocks are running with realistic propagation.
            if (clk.is_propagated) {
                const propBadge = document.createElement('span');
                propBadge.style.cssText =
                    'font-size:11px;padding:1px 6px;border-radius:3px;font-weight:600;' +
                    'background:var(--sdc-propagated-bg);color:var(--sdc-propagated-fg);';
                propBadge.textContent = 'propagated';
                propBadge.title = 'set_propagated_clock — timing uses computed ' +
                    'network delays instead of the default ideal (zero-delay) model';
                hdr.appendChild(propBadge);
            }
            // create_clock -add — clock was added to pins that already had a
            // clock rather than replacing the existing one. Surfaced so
            // multi-clock pins are obvious.
            if (clk.add_to_pins) {
                const addBadge = document.createElement('span');
                addBadge.style.cssText =
                    'font-size:11px;padding:1px 5px;border-radius:3px;font-weight:600;' +
                    'background:var(--bg-input);color:var(--fg-muted);' +
                    'border:1px dotted var(--border);';
                addBadge.textContent = '-add';
                addBadge.title = 'create_clock -add — this clock was added to ' +
                    'pins that already carry another clock';
                hdr.appendChild(addBadge);
            }
            // Generated-clock -combinational — master is reached through
            // combinational logic only (no register in between).
            if (clk.is_generated && clk.combinational) {
                const cbBadge = document.createElement('span');
                cbBadge.style.cssText =
                    'font-size:11px;padding:1px 5px;border-radius:3px;font-weight:600;' +
                    'background:var(--bg-input);color:var(--fg-muted);' +
                    'border:1px dotted var(--border);';
                cbBadge.textContent = 'comb';
                cbBadge.title = 'create_generated_clock -combinational — the ' +
                    'master clock is reached through combinational logic only';
                hdr.appendChild(cbBadge);
            }
            // Generated-clock "stale" diagnostic — STA has not (re-)computed
            // this generated clock's waveform from the master since the
            // last graph change. Period/waveform shown may be out of date;
            // hitting the toolbar's "Resolve generated" button refreshes.
            if (clk.is_generated && clk.generated_up_to_date === false) {
                const stBadge = document.createElement('span');
                stBadge.style.cssText =
                    'font-size:11px;padding:1px 5px;border-radius:3px;font-weight:600;' +
                    'background:var(--error);color:var(--fg-white);';
                stBadge.textContent = 'stale';
                stBadge.title = 'generated waveform is stale — click "Resolve ' +
                    'generated" in the toolbar to refresh from the master clock';
                hdr.appendChild(stBadge);
            }

            // Endpoints chip — number of timing-endpoint pins this clock
            // reaches. Populated lazily: if the cache is already filled
            // (the user previously hit the "list" link, or returns to
            // this tab after computing), the count renders inline. Until
            // then the chip shows `endpoints: ?` with a small (list)
            // link that triggers the count walk on demand. Keeping it
            // off the auto-load path matters on million-instance designs
            // where the underlying graph build is the slowest SDC call.
            const epChip = document.createElement('span');
            epChip.dataset.endpointsChip = '';
            epChip.style.cssText =
                'font-size:11px;padding:1px 6px;border-radius:3px;' +
                'font-weight:600;background:var(--bg-input);' +
                'color:var(--fg-muted);';
            this._renderEndpointChipContent(epChip, clk.name);
            hdr.appendChild(epChip);
            card.appendChild(hdr);

            // Body: stats left + waveform right
            const body = document.createElement('div');
            body.style.cssText = 'display:flex;';

            // Stats column
            const stats = document.createElement('div');
            stats.style.cssText =
                `width:${STATS_W}px;flex-shrink:0;padding:6px 8px;font-size:12px;` +
                'border-right:1px solid var(--border);color:var(--fg-primary);' +
                'display:flex;flex-direction:column;gap:2px;';

            const addStat = (label, value, color, tooltip) => {
                const row = document.createElement('div');
                row.style.cssText =
                    `display:flex;gap:4px;min-width:0;${color ? `color:${color};` : ''}`;
                if (tooltip) row.title = tooltip;
                const lbl = document.createElement('span');
                // Widened from 24px so "setup unc" / "hold unc" fit without
                // pushing the value column around.
                lbl.style.cssText = 'color:var(--fg-muted);min-width:60px;flex-shrink:0;';
                lbl.textContent = label;
                const val = document.createElement('span');
                val.style.cssText =
                    'font-family:monospace;overflow:hidden;text-overflow:ellipsis;' +
                    'white-space:nowrap;min-width:0;';
                val.textContent = value;
                val.title = tooltip ? `${tooltip}: ${value}` : value;
                row.appendChild(lbl); row.appendChild(val);
                stats.appendChild(row);
                return val;
            };

            // Period / frequency: when STA reports period == 0 (generated
            // clocks whose master hasn't been resolved yet, mostly), show
            // an em-dash instead of "0.000ns" / "Infinity GHz".
            const periodText = (Number.isFinite(clk.period) && clk.period > 0)
                ? `${clk.period.toPrecision(4)}${this._timeUnit}`
                : '—';
            addStat('period', periodText,
                undefined,
                'clock period (— means STA has not yet resolved a period for this clock)');
            addStat('freq',   this._periodToFreq(clk.period),
                undefined, 'operating frequency (derived from period)');
            // Duty cycle — derived from waveform: high-time / period.
            // 50% is the default for create_clock without an explicit
            // -waveform; values significantly off 50% suggest a custom
            // waveform that can affect setup/hold margins on either edge.
            if (Number.isFinite(clk.period) && clk.period > 0
                && Array.isArray(clk.waveform) && clk.waveform.length >= 2) {
                const high = clk.waveform[1] - clk.waveform[0];
                const duty = (high / clk.period) * 100;
                addStat('duty', `${duty.toPrecision(3)}%`, undefined,
                    'duty cycle = (fall − rise) / period');
            }
            if (clk.uncertainty_setup != null)
                addStat(TIMING_LABELS.setup_unc.short,
                        `${clk.uncertainty_setup.toPrecision(3)}${this._timeUnit}`,
                        'var(--sdc-text-setup)', TIMING_LABELS.setup_unc.tip);
            if (clk.uncertainty_hold != null)
                addStat(TIMING_LABELS.hold_unc.short,
                        `${clk.uncertainty_hold.toPrecision(3)}${this._timeUnit}`,
                        'var(--sdc-text-hold)', TIMING_LABELS.hold_unc.tip);

            // The verbose / wide stuff (source pin lists, generated-
            // clock relationship, edge arrays) used to be crammed
            // into the narrow 160px stats column on the left, where
            // long pin paths immediately overflowed.  It now lives in
            // a dedicated `details` section below the body so each
            // row gets the full card width.  The left-stats column
            // keeps just the compact numeric stats (period, freq,
            // duty, uncertainty); waveform stays on the right; the
            // `details` block opens up only when there's something
            // to show.
            //
            // (Created here so subsequent helper closures can capture
            //  `details`; appended to the card after `body` below.)
            const details = document.createElement('div');
            details.style.cssText =
                'display:none;padding:6px 10px;font-size:12px;'
                + 'border-top:1px solid var(--border-subtle);'
                + 'color:var(--fg-primary);'
                + 'flex-direction:column;gap:2px;';
            const addDetailRow = (label, value, tooltip) => {
                details.style.display = 'flex';
                const row = document.createElement('div');
                row.style.cssText =
                    'display:flex;gap:8px;align-items:baseline;min-width:0;';
                if (tooltip) row.title = tooltip;
                const lbl = document.createElement('span');
                lbl.style.cssText =
                    'color:var(--fg-muted);min-width:50px;flex-shrink:0;';
                lbl.textContent = label;
                row.appendChild(lbl);
                const val = document.createElement('span');
                val.style.cssText =
                    'flex:1;min-width:0;font-family:monospace;';
                if (typeof value === 'string') {
                    val.textContent = value;
                } else if (value) {
                    val.appendChild(value);
                }
                row.appendChild(val);
                details.appendChild(row);
                return val;
            };

            // Source: pin(s) the clock is defined on. Lives in the
            // details section so deep hierarchical paths can stretch
            // across the full card width.
            //   Virtual clocks have no pin at all.
            //   Generated clocks report a single src_pin (where the
            //     divider sits) plus the source pins of the create
            //     command (`sources`).
            //   Primary clocks report one or more source ports/pins in
            //     `sources`. Multi-source create_clock (common for
            //     divided clocks shared across subblocks, or
            //     `create_clock -add` to attach a new clock to an
            //     existing pin) used to collapse to "first (+N more)";
            //     we now render each one on its own row so the user
            //     can see — and click into — every anchor point.
            //
            //   Each row is path-truncated from the LEFT so the leaf
            //     (the pin name itself) stays readable when the
            //     hierarchy is deep. A small ⊞ prefix marks top-level
            //     ports vs internal pins.
            const sourceIcon = (odbType) => {
                if (odbType === 'bterm') return '⊞';   // top-level port
                if (odbType === 'iterm') return '◇';   // internal pin
                if (odbType === 'moditerm') return '◆'; // hierarchical pin
                return '';
            };
            const addPinDetail = (label, name, odbRef, tooltip) => {
                const v = addDetailRow(label, '', tooltip);
                v.style.color = 'var(--fg-muted)';
                v.style.overflow = 'hidden';
                v.style.textOverflow = 'ellipsis';
                v.style.whiteSpace = 'nowrap';
                v.style.direction = 'rtl';
                v.style.textAlign = 'left';
                const icon = odbRef ? sourceIcon(odbRef.odb_type) : '';
                v.textContent = icon ? `${icon} ${name}` : name;
                v.title = tooltip ? `${tooltip}: ${name}` : name;
                if (odbRef) this._linkifyPin(v, odbRef);
                return v;
            };
            if (clk.is_virtual) {
                addDetailRow('src', '(virtual)',
                    'virtual clock — no design pin; used as a timing '
                    + 'reference for set_input/output_delay only')
                    .style.color = 'var(--fg-muted)';
            } else {
                // Generated clocks emit `src_pin` separately from
                // `sources` (which is the create_generated_clock's
                // attachment pins). Both are useful — one shows where
                // the clock is *derived from* (the upstream master),
                // the other shows where the new clock attaches.
                if (clk.is_generated && clk.src_pin) {
                    addPinDetail(
                        'src',
                        clk.src_pin,
                        { odb_type: clk.src_pin_odb_type,
                          odb_id:   clk.src_pin_odb_id },
                        'create_generated_clock -source — pin the '
                        + 'generated clock is derived from');
                }
                const sources = Array.isArray(clk.sources) ? clk.sources : [];
                const sourcesOdb = Array.isArray(clk.sources_odb)
                    ? clk.sources_odb : [];
                for (let i = 0; i < sources.length; ++i) {
                    // For primary clocks `at` IS the source. For
                    // generated clocks `at` is the attachment pin
                    // (where the divider's output lives), distinct
                    // from the master `src` above.
                    const label = (i === 0)
                        ? (clk.is_generated ? 'at' : 'src')
                        : '';  // continuation row, blank label
                    addPinDetail(
                        label,
                        sources[i],
                        sourcesOdb[i] || null,
                        clk.is_generated
                            ? 'create_generated_clock attachment pin'
                            : 'create_clock anchor pin');
                }
            }
            // Generated-clock relationship to its master:
            //   - "from <master>"
            //   - one or more transformation tags: ÷N, ×N, inv, -edges, -edge_shifts
            // Surfaced together so the user can tell how the generated clock
            // is derived (a divide_by_2 vs. a multiply_by_4 of the same master
            // are wildly different). Lives in the details section
            // alongside the source pins so the relationship reads as
            // a coherent block.
            if (clk.is_generated) {
                if (clk.master_clock) {
                    // Append "(inferred)" suffix when STA picked the master
                    // clock from the graph rather than the user specifying
                    // it explicitly with -master_clock — handy for tracking
                    // down "wait, why is THIS the master?" situations.
                    const masterTxt = clk.master_inferred
                        ? `${clk.master_clock} (inferred)`
                        : clk.master_clock;
                    addDetailRow('from', masterTxt,
                        clk.master_inferred
                            ? 'master clock — STA inferred this from the design ' +
                              '(no -master_clock on create_generated_clock)'
                            : 'master clock this generated clock derives from');
                }
                const tags = [];
                if (clk.divide_by   != null && clk.divide_by   > 1) tags.push(`÷${clk.divide_by}`);
                if (clk.multiply_by != null && clk.multiply_by > 1) tags.push(`×${clk.multiply_by}`);
                if (clk.invert) tags.push('inverted');
                if (Array.isArray(clk.edges) && clk.edges.length > 0)
                    tags.push(`-edges {${clk.edges.join(' ')}}`);
                if (Array.isArray(clk.edge_shifts) && clk.edge_shifts.length > 0) {
                    tags.push('-edge_shift {' + clk.edge_shifts
                        .map(s => s.toPrecision(3))
                        .join(' ') + this._timeUnit + '}');
                }
                if (tags.length > 0) {
                    addDetailRow('by', tags.join('  '),
                        'transformation applied to the master clock');
                }
            }
            body.appendChild(stats);

            // ── Waveform SVG ─────────────────────────────────────────────
            // Bail out gracefully when the period isn't available — divisions
            // by zero produce a blank, broken-looking SVG (Infinity scaling)
            // and the period ticks below would all stack at x=0.  Instead
            // render a centered explanatory message so it's clear *why* the
            // diagram is missing.
            const svg = this._makeSvg(WAVE_W, WAVE_H);
            const havePeriod = Number.isFinite(clk.period) && clk.period > 0;
            if (!havePeriod) {
                this._svgText(svg, WAVE_W / 2, WAVE_H / 2 + 4,
                    clk.is_generated
                        ? 'period unresolved — STA hasn\'t computed it yet'
                        : 'no period — virtual or undefined clock',
                    'var(--fg-muted)', 10, 'middle');
            } else {
                const displayDuration = clk.period * 3;
                this._drawWaveform(svg, clk, 0, 0, DRAW_W, ROW_H, displayDuration, false);

                // Axis line (stops 2px before right edge to match DRAW_W)
                this._svgLine(svg, 0, AX_Y, DRAW_W, AX_Y, 'var(--canvas-axis)', 1);

                // Period boundary ticks: 0T, 1T, 2T, 3T
                for (let i = 0; i <= 3; i++) {
                    const t = i * clk.period;
                    const x = tx(t, displayDuration);
                    const tip = i === 0
                        ? `start (t = 0)`
                        : `period boundary — ${i} × T = ${
                            t.toPrecision(3)}${this._timeUnit}`;
                    const tickLn = this._svgLine(svg, x, AX_Y - 3, x,
                        AX_Y + 3, 'var(--canvas-axis)', 1);
                    this._svgTitle(tickLn, tip);
                    const anchor = i === 0 ? 'start' : i === 3 ? 'end' : 'middle';
                    const xLbl  = i === 0 ? x + 2 : i === 3 ? x - 2 : x;
                    const lbl = this._svgText(svg, xLbl, LABEL_Y,
                        `${t.toPrecision(3)}${this._timeUnit}`,
                        'var(--canvas-label)', 7.5, anchor);
                    this._svgTitle(lbl, tip);
                }

                // Waveform-edge timestamps: small ticks + time labels at each transition
                if (clk.waveform && clk.waveform.length >= 2) {
                    const MIN_SPACING = 28;  // pixels between consecutive edge labels
                    let prevLabelX = -MIN_SPACING;

                    for (let p = 0; p < 3; p++) {
                        for (let i = 0; i < clk.waveform.length; i++) {
                            const edgeT = p * clk.period + clk.waveform[i];
                            if (edgeT >= displayDuration) break;
                            const ex = tx(edgeT, displayDuration);
                            const isRise = (i % 2 === 0);
                            const edgeTip =
                                `${isRise ? 'rising' : 'falling'} edge @ ` +
                                `${edgeT.toPrecision(3)}${this._timeUnit} ` +
                                `(period ${p}, t = ${
                                    clk.waveform[i].toPrecision(3)}${
                                    this._timeUnit} within period)`;

                            // Minor tick at the edge — clickable hover surface
                            // for the per-edge tooltip.
                            const tickLn = this._svgLine(svg, ex, AX_Y - 2,
                                ex, AX_Y + 2, 'var(--fg-muted)', 0.8);
                            this._svgTitle(tickLn, edgeTip);

                            // Label only when not too close to a period-boundary tick or prior label
                            const nearPeriod = [0, 1, 2, 3].some(
                                j => Math.abs(ex - tx(j * clk.period, displayDuration)) < 22);
                            if (!nearPeriod && ex - prevLabelX >= MIN_SPACING && ex < DRAW_W - 16) {
                                const lbl = this._svgText(svg, ex, LABEL_Y,
                                    `${edgeT.toPrecision(3)}${this._timeUnit}`,
                                    'var(--fg-muted)', 7, 'middle');
                                this._svgTitle(lbl, edgeTip);
                                prevLabelX = ex;
                            }
                        }
                    }
                }
            }

            const waveDiv = document.createElement('div');
            waveDiv.style.cssText = 'flex:1;overflow:hidden;background:var(--bg-main);';
            waveDiv.appendChild(svg);
            body.appendChild(waveDiv);
            card.appendChild(body);

            // Details section (sources / master relationship / edge
            // arrays) populated by addDetailRow above. Auto-hidden
            // when no row was added (`details.style.display` flips to
            // 'flex' on the first call).
            card.appendChild(details);

            // SDC -comment from create_clock, if any.
            this._appendComment(card, clk.comment);

            // Tree mode: this card owns a children-container that's indented
            // and carries a vertical guide line on its left edge so children
            // visibly hang off their parent. The line is dashed and uses the
            // theme's border color so it stays subtle.
            const hasChildren = node.children && node.children.length > 0;
            if (useTree && hasChildren) {
                const wrapper = document.createElement('div');
                wrapper.style.cssText = 'margin-bottom:8px;';
                // Strip the card's own bottom margin since the wrapper owns it.
                card.style.marginBottom = '4px';
                wrapper.appendChild(card);
                const childContainer = document.createElement('div');
                childContainer.style.cssText =
                    'margin-left:14px;padding-left:12px;' +
                    'border-left:1px dashed var(--border);';
                wrapper.appendChild(childContainer);
                parentContainer.appendChild(wrapper);
                recurseInto(childContainer);
            } else {
                parentContainer.appendChild(card);
                // Flatten mode (or no children): keep recursing into the same
                // outer container so every visible card sits at the top level.
                recurseInto(parentContainer);
            }
        };

        for (const root of this._clockTree) visitNode(root, this._cardScrollArea);
    }

    // ── Legacy tree helpers (kept for data traversal) ────────────────────────

    _makeTreeNode(node, depth) {
        const clk = this._clockMap[node.name];
        const hasChildren = node.children && node.children.length > 0;

        const wrapper = document.createElement('div');

        const row = document.createElement('div');
        row.style.cssText =
            `display:flex;align-items:center;padding:3px 6px 3px ${8 + depth * 16}px;` +
            'cursor:pointer;user-select:none;white-space:nowrap;';
        row.addEventListener('mouseenter', () => {
            if (this._selectedClockName !== node.name)
                row.style.background = 'var(--bg-hover)';
        });
        row.addEventListener('mouseleave', () => {
            if (this._selectedClockName !== node.name)
                row.style.background = '';
        });
        row.addEventListener('click', () => this._selectClock(node.name));
        this['_row_' + node.name] = row;  // for selection highlight

        // Collapse toggle
        const toggle = document.createElement('span');
        toggle.style.cssText = 'width:14px;display:inline-block;color:var(--fg-muted);font-size:11px;';
        toggle.textContent = hasChildren ? '▶' : '';
        row.appendChild(toggle);

        // Clock name — clock names occasionally include hierarchical
        // prefixes when generated from a hierarchical pin, so left-
        // truncate to keep the leaf visible.
        const nameSpan = document.createElement('span');
        nameSpan.style.cssText =
            'flex:1;color:var(--fg-primary);' + TRUNCATE_PATH_CSS;
        nameSpan.textContent = node.name;
        nameSpan.title = node.name;
        row.appendChild(nameSpan);

        // Period annotation
        if (clk) {
            const period = document.createElement('span');
            period.style.cssText = 'margin-left:6px;color:var(--fg-muted);font-size:12px;';
            period.textContent = `${clk.period.toPrecision(4)}${this._timeUnit}  ${this._periodToFreq(clk.period)}`;
            row.appendChild(period);

            // Relationship label for generated clocks
            if (clk.is_generated) {
                const rel = document.createElement('span');
                rel.style.cssText =
                    'margin-left:4px;padding:1px 4px;border-radius:3px;font-size:11px;' +
                    'background:var(--bg-header);color:var(--accent-tab);';
                if (clk.divide_by > 1) {
                    rel.textContent = `÷${clk.divide_by}`;
                } else if (clk.multiply_by > 1) {
                    rel.textContent = `×${clk.multiply_by}`;
                } else if (clk.invert) {
                    rel.textContent = 'inv';
                } else {
                    rel.textContent = 'gen';
                }
                row.appendChild(rel);
            }
        }

        wrapper.appendChild(row);

        // Children container (collapsible)
        if (hasChildren) {
            const childContainer = document.createElement('div');
            childContainer.style.display = 'none';
            let collapsed = true;

            toggle.addEventListener('click', (e) => {
                e.stopPropagation();
                collapsed = !collapsed;
                childContainer.style.display = collapsed ? 'none' : 'block';
                toggle.textContent = collapsed ? '▶' : '▼';
            });

            for (const child of node.children) {
                childContainer.appendChild(this._makeTreeNode(child, depth + 1));
            }
            wrapper.appendChild(childContainer);
        }

        return wrapper;
    }

    _selectClock(name) { /* no-op: replaced by card layout */ }

    // ── Waveform rendering ───────────────────────────────────────────────────

    // Collect the ancestor chain: root → ... → selected clock.
    _ancestors(name) {
        const chain = [];
        let current = name;
        while (current) {
            const clk = this._clockMap[current];
            if (!clk) break;
            chain.unshift(clk);
            current = clk.master_clock;
        }
        return chain;
    }

    // Collect all descendants of a clock (recursive).
    _descendants(treeNode) {
        const result = [];
        const traverse = (node) => {
            const clk = this._clockMap[node.name];
            if (clk) result.push(clk);
            if (node.children) node.children.forEach(traverse);
        };
        traverse(treeNode);
        return result;
    }

    // Find the tree node for a clock name.
    _findTreeNode(name, nodes) {
        for (const node of (nodes || this._clockTree)) {
            if (node.name === name) return node;
            if (node.children) {
                const found = this._findTreeNode(name, node.children);
                if (found) return found;
            }
        }
        return null;
    }

    _renderWaveforms(selectedName) {
        /* no-op: replaced by _renderClockCards */
        return;

        // Build the set of clocks to display: ancestors + selected + all descendants.
        const ancestors = this._ancestors(selectedName);
        const selectedTreeNode = this._findTreeNode(selectedName);
        const descendants = selectedTreeNode ? this._descendants(selectedTreeNode) : [];

        // ancestors already includes selected; remove duplicates preserving order.
        const seenNames = new Set(ancestors.map(c => c.name));
        for (const d of descendants) {
            if (!seenNames.has(d.name)) {
                seenNames.add(d.name);
                ancestors.push(d);
            }
        }
        const family = ancestors;  // ordered: root → selected → descendants

        if (family.length === 0) {
            this._wavePane.innerHTML =
                '<div style="padding:16px;color:var(--fg-muted);">No clock data.</div>';
            return;
        }

        // Determine the display time range.
        // Show enough cycles so the deepest-frequency clock shows ≥2 full cycles.
        const rootPeriod = family[0].period;
        const maxPeriod = Math.max(...family.map(c => c.period));
        // Show enough of the root clock's time axis to fit ≥2 cycles of the slowest derived clock.
        const displayCycles = Math.max(2, Math.ceil(maxPeriod / rootPeriod) + 1);
        const displayDuration = rootPeriod * displayCycles;

        const SVG_W = Math.max(500, this._wavePane.clientWidth - 220);
        const ROW_H = 36;
        const LABEL_W = 180;
        const WAVE_W = SVG_W - LABEL_W - 8;
        const PADDING_TOP = 4;
        // Axis sits 8px below the last waveform row; labels are 14px below the
        // axis line, so we need at least 8 + 14 + font(9) ≈ 30px of bottom room.
        const BOTTOM_PAD = 30;

        const axisY = PADDING_TOP + family.length * ROW_H + 8;
        const svgH = axisY + BOTTOM_PAD;
        const svg = this._makeSvg(SVG_W, svgH);

        // X axis line
        this._svgLine(svg, LABEL_W, axisY, LABEL_W + WAVE_W, axisY, 'var(--canvas-axis)', 1);

        // Tick marks + labels
        for (let i = 0; i <= displayCycles; i++) {
            const t = (i * rootPeriod / displayDuration) * WAVE_W + LABEL_W;
            this._svgLine(svg, t, axisY - 3, t, axisY + 4, 'var(--canvas-axis)', 1);
            this._svgText(
                svg, t, axisY + 15,
                `${(i * rootPeriod).toPrecision(3)}`, 'var(--canvas-label)', 9, 'middle'
            );
        }

        // Unit label: right-aligned just above the axis end tick, inside SVG bounds.
        this._svgText(svg, LABEL_W + WAVE_W, axisY - 6,
            `(${this._timeUnit})`, 'var(--canvas-label)', 9, 'end');

        // One waveform row per clock
        family.forEach((clk, rowIdx) => {
            const y0 = PADDING_TOP + rowIdx * ROW_H;
            const isSelected = clk.name === selectedName;

            // Highlight row background for selected clock
            if (isSelected) {
                const rect = document.createElementNS(SVG_NS, 'rect');
                rect.setAttribute('x', 0);
                rect.setAttribute('y', y0);
                rect.setAttribute('width', SVG_W);
                rect.setAttribute('height', ROW_H);
                rect.setAttribute('fill', 'var(--bg-selected-row)');
                rect.setAttribute('fill-opacity', '0.4');
                svg.appendChild(rect);
            }

            // Label area
            const labelX = 4;
            const labelY = y0 + ROW_H / 2 + 4;
            this._svgText(svg, labelX, labelY,
                clk.name, isSelected ? 'var(--fg-bright)' : 'var(--fg-primary)', 11, 'start');

            // Period + uncertainty annotation below name
            let periodLine = `${clk.period.toPrecision(4)}${this._timeUnit}  ${this._periodToFreq(clk.period)}`;
            if (clk.is_generated) {
                let relStr = '';
                if (clk.divide_by > 1) relStr = `÷${clk.divide_by}`;
                else if (clk.multiply_by > 1) relStr = `×${clk.multiply_by}`;
                else if (clk.invert) relStr = 'inv';
                else relStr = 'gen';
                periodLine += `  ${relStr}`;
            }
            const uncParts = [];
            if (clk.uncertainty_setup != null)
                uncParts.push(`${TIMING_LABELS.setup_unc.short}=${clk.uncertainty_setup.toPrecision(3)}`);
            if (clk.uncertainty_hold != null)
                uncParts.push(`${TIMING_LABELS.hold_unc.short}=${clk.uncertainty_hold.toPrecision(3)}`);
            if (uncParts.length) periodLine += `  ${uncParts.join(' ')}`;
            this._svgText(svg, labelX, labelY + 13,
                periodLine, 'var(--fg-muted)', 9, 'start');

            // Waveform
            this._drawWaveform(svg, clk, y0, LABEL_W, WAVE_W, ROW_H, displayDuration, isSelected);
        });

        this._wavePane.appendChild(svg);
    }

    // Draw a digital waveform for one clock row.
    _drawWaveform(svg, clk, y0, xOff, waveW, rowH, displayDuration, isSelected) {
        const waveform = clk.waveform;  // [rise0, fall0, rise1, fall1, ...]  in time units
        if (!waveform || waveform.length < 2) return;

        const period = clk.period;
        const color = isSelected ? 'var(--accent-tab)' : 'var(--fg-secondary)';
        const HIGH_Y = y0 + 6;
        const LOW_Y  = y0 + rowH - 10;

        // Map time → x pixel
        const tx = (t) => xOff + (t / displayDuration) * waveW;

        // Draw uncertainty bands at each edge before the waveform path (renders behind)
        const setupUnc = clk.uncertainty_setup;
        const holdUnc  = clk.uncertainty_hold;
        if (setupUnc != null || holdUnc != null) {
            let t = 0;
            while (t < displayDuration) {
                for (let i = 0; i < waveform.length; i++) {
                    const edgeT = t + waveform[i];
                    if (edgeT > displayDuration) break;
                    const ex = tx(edgeT);
                    const bandH = LOW_Y - HIGH_Y;

                    // Theme-aware band fills: use the shared waveform palette
                    // with partial opacity (0.38 selected, 0.19 unselected —
                    // same ratios as the old 8-digit #RRGGBBAA literals).
                    if (setupUnc != null && setupUnc > 0) {
                        const bw = Math.max(1, (setupUnc / displayDuration) * waveW);
                        const rect = this._svgRect(svg, ex - bw, HIGH_Y, bw,
                                                   bandH,
                                                   'var(--sdc-wf-setup-unc)',
                                                   isSelected ? 0.5 : 0.3);
                        if (rect) {
                            rect.dataset.band = 'setup-unc';
                            this._svgTitle(rect,
                                `setup uncertainty = ${setupUnc.toPrecision(3)}` +
                                `${this._timeUnit} reserved before edge @ ` +
                                `${edgeT.toPrecision(3)}${this._timeUnit}`);
                        }
                    }
                    if (holdUnc != null && holdUnc > 0) {
                        const bw = Math.max(1, (holdUnc / displayDuration) * waveW);
                        const rect = this._svgRect(svg, ex, HIGH_Y, bw, bandH,
                                                   'var(--sdc-wf-hold-unc)',
                                                   isSelected ? 0.5 : 0.3);
                        if (rect) {
                            rect.dataset.band = 'hold-unc';
                            this._svgTitle(rect,
                                `hold uncertainty = ${holdUnc.toPrecision(3)}` +
                                `${this._timeUnit} reserved after edge @ ` +
                                `${edgeT.toPrecision(3)}${this._timeUnit}`);
                        }
                    }
                }
                t += period;
            }
        }

        // Build the per-period edge list, then delegate to the shared helper.
        const edges = [];
        for (let t = 0; t < displayDuration; t += period) {
            for (let i = 0; i < waveform.length; i++) {
                const edgeT = t + waveform[i];
                if (edgeT > displayDuration) break;
                edges.push({ t: edgeT, isRise: (i % 2 === 0) });
            }
        }
        if (!edges.length) return;
        const startY = waveform[0] > 0 ? LOW_Y : HIGH_Y;
        const path = this._drawClockPath(svg, edges, tx, HIGH_Y, LOW_Y,
                                          startY, /*tStart=*/ 0,
                                          /*tEnd=*/ displayDuration,
                                          color, isSelected ? 1.5 : 1);
        if (!path) return;

        // Fill area under HIGH signal (subtle) — reuse the path's d-string
        // and close it down to LOW_Y for a translucent shaded region.
        const fillColor = isSelected ? 'var(--accent-tab)' : 'var(--fg-muted)';
        const fillPath = document.createElementNS(
            SVG_NS, 'path');
        fillPath.setAttribute('d',
            path.getAttribute('d')
            + ` L ${tx(displayDuration)} ${LOW_Y} L ${tx(0)} ${LOW_Y} Z`);
        fillPath.setAttribute('fill', fillColor);
        fillPath.setAttribute('fill-opacity', '0.07');
        svg.appendChild(fillPath);
    }

    // ── Case-analysis panel (own tab) ───────────────────────────────────────

    _renderCaseStrip() {
        if (!this._caseStrip) return;
        this._caseStrip.innerHTML = '';

        const caseCount = this._caseAnalysis.filter(e => e.kind === 'case').length;
        const logicCount = this._caseAnalysis.filter(e => e.kind === 'logic').length;

        // Mode context: the active SDC mode drives which case_analysis pins are
        // in effect.  Shown even when the constraint list is empty so the user
        // isn't left wondering which mode they're looking at.
        if (this._currentMode) {
            const modeHdr = document.createElement('div');
            modeHdr.style.cssText =
                'padding:6px 10px;font-size:12px;font-weight:600;color:var(--fg-muted);' +
                'border-bottom:1px solid var(--border-subtle);' +
                'letter-spacing:0.04em;text-transform:uppercase;';
            modeHdr.textContent = `Mode: ${this._currentMode}`;
            this._caseStrip.appendChild(modeHdr);
        }

        if (this._caseAnalysis.length === 0) {
            const msg = document.createElement('div');
            msg.style.cssText =
                'padding:12px;color:var(--fg-muted);font-style:italic;';
            msg.textContent =
                'No set_case_analysis or set_logic_zero/one/dc constraints defined.';
            this._caseStrip.appendChild(msg);
            return;
        }

        // Summary header listing the breakdown by source command.
        const header = document.createElement('div');
        header.style.cssText =
            'padding:6px 10px;font-size:12px;color:var(--fg-muted);' +
            'border-bottom:1px solid var(--border-subtle);margin-bottom:4px;';
        const pieces = [];
        if (caseCount  > 0) pieces.push(`${caseCount} set_case_analysis`);
        if (logicCount > 0) pieces.push(`${logicCount} set_logic_zero/one/dc`);
        header.textContent = pieces.join(' · ');
        this._caseStrip.appendChild(header);

        // Grid layout so the kind badge / value badge / pin path
        // columns line up identically on every row, regardless of
        // whether the source command was set_case_analysis ("case",
        // 4 chars) or set_logic_* ("logic", 5 chars). Flex with a
        // gap let the kind badge take its natural width, so the
        // value-badge column shifted left/right between rows.
        const table = document.createElement('div');
        table.style.cssText =
            'padding:2px 0;'
            + 'display:grid;grid-template-columns:auto auto 1fr;'
            + 'column-gap:8px;row-gap:0;align-items:center;';

        for (const entry of this._caseAnalysis) {
            const { pin, value, kind } = entry;
            // Each row is three grid items; the surrounding `display:
            // grid` on `table` lays them out in three vertical
            // columns. Use `display:contents` on a wrapper if we
            // need per-row hover effects later; today rows just
            // share the parent's grid tracks directly.
            //
            // Bottom border lives on the LAST cell only so it spans
            // the full row width, matching the prior flex layout.
            const baseCell =
                'padding:2px 0;border-bottom:1px solid var(--border-subtle);';

            // Small source tag distinguishing set_case_analysis from set_logic_*
            const kindBadge = document.createElement('span');
            kindBadge.style.cssText = baseCell
                + 'padding-left:8px;'
                + 'display:flex;align-items:center;';
            const kindPill = document.createElement('span');
            kindPill.style.cssText =
                'display:inline-block;font-size:11px;padding:0 4px;border-radius:2px;' +
                'color:var(--fg-muted);background:var(--bg-input);' +
                'font-family:monospace;letter-spacing:0.03em;'
                // Fixed min-width so "case" and "logic" pills are
                // visually identical width.
                + 'min-width:32px;text-align:center;';
            kindPill.textContent = kind === 'logic' ? 'logic' : 'case';
            kindPill.title = kind === 'logic'
                ? 'set_logic_zero / set_logic_one / set_logic_dc'
                : 'set_case_analysis';
            kindBadge.appendChild(kindPill);

            const badge = document.createElement('span');
            const badgeColor = value === '0' ? '#a0c4ff'
                : value === '1' ? '#b9f6ca'
                : value === 'rise' ? '#ffd180'
                : value === 'fall' ? '#ff8a80'
                : 'var(--fg-muted)';
            badge.style.cssText = baseCell
                + 'display:flex;align-items:center;';
            const valuePill = document.createElement('span');
            valuePill.style.cssText =
                `display:inline-block;width:36px;text-align:center;`
                + `font-size:12px;font-weight:700;`
                + `color:#000;background:${badgeColor};border-radius:3px;`
                + `padding:0 2px;`;
            valuePill.textContent = value;
            badge.appendChild(valuePill);

            const pinSpan = document.createElement('span');
            pinSpan.style.cssText = baseCell
                + 'padding-right:8px;'
                + 'font-family:monospace;font-size:12px;color:var(--fg-primary);'
                + 'min-width:0;'
                + TRUNCATE_PATH_CSS;
            pinSpan.title = pin;
            pinSpan.textContent = pin;
            // Each strip entry carries pin_odb_type/id from sdc_clock_modes.
            this._linkifyPin(pinSpan, entry, 'pin');

            table.appendChild(kindBadge);
            table.appendChild(badge);
            table.appendChild(pinSpan);
        }
        this._caseStrip.appendChild(table);
    }

    // ── Limits panel ─────────────────────────────────────────────────────────

    _buildLimitsPanel(container) {
        container.style.cssText = 'display:flex;flex-direction:column;height:100%;overflow:hidden;';

        const toolbar = document.createElement('div');
        toolbar.style.cssText =
            'display:flex;align-items:center;padding:2px 6px;gap:6px;border-bottom:1px solid var(--border);' +
            'background:var(--bg-header);flex-shrink:0;';
        const refreshBtn = document.createElement('button');
        refreshBtn.textContent = '↺ Refresh';
        refreshBtn.title = 'Reload SDC limits from the current design';
        refreshBtn.style.cssText =
            'padding:2px 8px;font-size:12px;cursor:pointer;background:var(--bg-input);' +
            'color:var(--fg-primary);border:1px solid var(--border);border-radius:3px;';
        refreshBtn.addEventListener('click', () => {
            this._limLoaded = false;
            this._limLoading = false;
            this._loadModes();
            this._loadLimits();
        });
        toolbar.appendChild(refreshBtn);
        container.appendChild(toolbar);

        const scroll = document.createElement('div');
        scroll.style.cssText = 'flex:1;overflow-y:auto;padding:8px;background:var(--bg-main);';
        this._limScrollArea = scroll;
        container.appendChild(scroll);
        scroll.innerHTML =
            '<div style="padding:12px;color:var(--fg-muted);font-style:italic;">Loading…</div>';
    }

    async _loadLimits() {
        if (this._limLoaded || this._limLoading) return;
        this._limLoading = true;
        this._limScrollArea.innerHTML =
            '<div style="padding:12px;color:var(--fg-muted);font-style:italic;">Loading…</div>';
        try {
            await this._app.websocketManager.readyPromise;
            const data = await this._requestWithTimeout({ type: 'sdc_limits' });
            this._renderLimits(data);
            this._limLoaded = true;
        } catch (e) {
            console.error('[SDC] limits load error:', e);
            this._showLoadError(this._limScrollArea, 'limits', e, () => {
                this._limLoaded = false;
                this._limLoading = false;
                this._loadLimits();
            });
        } finally {
            this._limLoading = false;
        }
    }

    _renderLimits(data) {
        this._limScrollArea.innerHTML = '';
        const tu = data.time_unit || 'ns';
        const cu = data.cap_unit  || 'pF';

        const latencies      = data.clock_latencies     || [];
        const insertions     = data.clock_insertions    || [];
        const uncertainties  = data.clock_uncertainties || [];
        const portLoads      = data.port_loads          || [];
        const disables       = data.disabled_timing     || [];
        const cellLimits     = data.cell_limits         || [];
        const clkSlewLimits  = data.clock_slew_limits   || [];
        const pinCapLimits   = data.pin_cap_limits      || [];
        const pinUncerts     = data.pin_clock_uncertainties || [];

        const hasAny = latencies.length || insertions.length
            || uncertainties.length || portLoads.length || disables.length
            || cellLimits.length || clkSlewLimits.length
            || pinCapLimits.length || pinUncerts.length;
        if (!hasAny) {
            this._limScrollArea.innerHTML =
                '<div style="padding:12px;color:var(--fg-muted);font-style:italic;">' +
                'No SDC limits defined (no set_clock_latency, set_clock_uncertainty, ' +
                'set_load, or set_disable_timing).</div>';
            return;
        }

        if (uncertainties.length > 0) {
            this._limScrollArea.appendChild(
                this._makeLimSection('Clock Uncertainties',
                    ['Clock', 'Setup', 'Hold'],
                    uncertainties.map(u => [
                        u.clock,
                        u.setup != null ? `${u.setup.toPrecision(3)} ${tu}` : '—',
                        u.hold  != null ? `${u.hold.toPrecision(3)} ${tu}`  : '—',
                    ])));
        }
        // Pin-anchored uncertainty (set_clock_uncertainty -to <pin> …) —
        // rendered alongside the per-clock table above. Only populated
        // when the timing graph has been built (gated like pin_cap_limits).
        if (pinUncerts.length > 0) {
            this._limScrollArea.appendChild(
                this._makeLimSection(
                    'Pin-Anchored Uncertainty  (set_clock_uncertainty -to <pin>)',
                    ['Pin', 'Setup', 'Hold'],
                    pinUncerts.map(u => [
                        u.pin,
                        u.setup != null ? `${u.setup.toPrecision(3)} ${tu}` : '—',
                        u.hold  != null ? `${u.hold.toPrecision(3)} ${tu}`  : '—',
                    ])));
        }

        if (latencies.length > 0) {
            this._limScrollArea.appendChild(
                this._makeLimSection('Clock Latencies',
                    ['Clock', 'Pin', 'Rise max', 'Rise min', 'Fall max', 'Fall min'],
                    latencies.map(l => [
                        l.clock || '—',
                        l.pin   || '—',
                        l.rise_max != null ? `${l.rise_max.toPrecision(3)} ${tu}` : '—',
                        l.rise_min != null ? `${l.rise_min.toPrecision(3)} ${tu}` : '—',
                        l.fall_max != null ? `${l.fall_max.toPrecision(3)} ${tu}` : '—',
                        l.fall_min != null ? `${l.fall_min.toPrecision(3)} ${tu}` : '—',
                    ])));
        }

        if (insertions.length > 0) {
            // Show all 8 cells of the (rise/fall × min/max × early/late)
            // product — set_clock_latency -source supports independent
            // values for each combination, and collapsing them hid that
            // detail. Cell shows the value directly when present, em-dash
            // otherwise. "late" and "early" sit in adjacent columns so the
            // user can spot asymmetry without a table tour.
            const fmt = (v) => v != null ? `${v.toPrecision(3)} ${tu}` : '—';
            this._limScrollArea.appendChild(
                this._makeLimSection('Clock Insertion Delays  (set_clock_latency -source)',
                    ['Clock', 'Pin',
                     'Late rise max',  'Late rise min',
                     'Late fall max',  'Late fall min',
                     'Early rise max', 'Early rise min',
                     'Early fall max', 'Early fall min'],
                    insertions.map(ins => [
                        ins.clock || '—',
                        ins.pin   || '—',
                        fmt(ins.late_rise_max),
                        fmt(ins.late_rise_min),
                        fmt(ins.late_fall_max),
                        fmt(ins.late_fall_min),
                        fmt(ins.early_rise_max),
                        fmt(ins.early_rise_min),
                        fmt(ins.early_fall_max),
                        fmt(ins.early_fall_min),
                    ])));
        }

        // ── Port loads (set_load only) ────────────────────────────────────
        // Pin/wire capacitance attached to top-level ports via set_load. The
        // set_max_* limits that used to live in this table are split out
        // into their own group below so the user can scan all "max-*"
        // constraints together.
        const portLoadsHaveLoad = portLoads.some(p =>
            p.rise_max != null || p.wire_cap_max != null);
        if (portLoadsHaveLoad) {
            this._limScrollArea.appendChild(
                this._makeLimSection('Port Loads  (set_load)',
                    ['Port', 'Pin cap (rise max)', 'Wire cap max'],
                    portLoads
                        .filter(p => p.rise_max != null || p.wire_cap_max != null)
                        .map(p => [
                            p.port,
                            p.rise_max     != null ? `${p.rise_max.toPrecision(3)} ${cu}` : '—',
                            p.wire_cap_max != null ? `${p.wire_cap_max.toPrecision(3)} ${cu}` : '—',
                        ])));
        }

        // ── Slew / Capacitance / Fanout limits group ─────────────────────
        //   Port-scope    : set_max_*  [get_ports …]
        //   Design-wide   : set_max_*  [current_design]
        //   Clock-scope   : set_max_transition -clock_path/-data_path -clock
        // All three render as their own table under a shared group header
        // so the user sees them together instead of hunting across the tab.
        const portLoadsHaveMax = portLoads.some(p =>
            p.slew_max != null || p.cap_limit != null || p.fanout_limit != null);
        const haveAnyMax = portLoadsHaveMax || cellLimits.length > 0
            || clkSlewLimits.length > 0 || pinCapLimits.length > 0;
        if (haveAnyMax) {
            this._limScrollArea.appendChild(this._makeLimGroupHeader(
                'Slew / Capacitance / Fanout Limits  (set_max_*)',
                'set_max_transition / set_max_capacitance / set_max_fanout — ' +
                'shown by scope (port → design → clock).'));
        }
        if (portLoadsHaveMax) {
            this._limScrollArea.appendChild(
                this._makeLimSection('Port-Scope Limits  ([get_ports …])',
                    ['Port', 'Slew max', 'Cap limit', 'Fanout limit'],
                    portLoads
                        .filter(p => p.slew_max != null || p.cap_limit != null
                                  || p.fanout_limit != null)
                        .map(p => [
                            p.port,
                            p.slew_max   != null ? `${p.slew_max.toPrecision(3)} ${tu}`  : '—',
                            p.cap_limit  != null ? `${p.cap_limit.toPrecision(3)} ${cu}` : '—',
                            p.fanout_limit != null ? `${p.fanout_limit}`                  : '—',
                        ])));
        }
        if (cellLimits.length > 0) {
            this._limScrollArea.appendChild(
                this._makeLimSection('Design-Wide Limits  ([current_design])',
                    ['Scope', 'Cell', 'Slew max', 'Slew min', 'Cap limit', 'Fanout limit'],
                    cellLimits.map(l => [
                        l.scope || 'design',
                        l.name  || '—',
                        l.slew_max     != null ? `${l.slew_max.toPrecision(3)} ${tu}`     : '—',
                        l.slew_min     != null ? `${l.slew_min.toPrecision(3)} ${tu}`     : '—',
                        l.cap_limit    != null ? `${l.cap_limit.toPrecision(3)} ${cu}`    : '—',
                        l.fanout_limit != null ? `${l.fanout_limit}`                       : '—',
                    ])));
        }
        if (clkSlewLimits.length > 0) {
            const cell = (v) => v != null ? `${v.toPrecision(3)} ${tu}` : '—';
            this._limScrollArea.appendChild(
                this._makeLimSection('Clock-Scope Slew Limits  (-clock_path / -data_path -clock)',
                    ['Clock',
                     'Clk rise max',  'Clk rise min',
                     'Clk fall max',  'Clk fall min',
                     'Data rise max', 'Data rise min',
                     'Data fall max', 'Data fall min'],
                    clkSlewLimits.map(c => [
                        c.clock || '—',
                        cell(c.clk_rise_max),  cell(c.clk_rise_min),
                        cell(c.clk_fall_max),  cell(c.clk_fall_min),
                        cell(c.data_rise_max), cell(c.data_rise_min),
                        cell(c.data_fall_max), cell(c.data_fall_min),
                    ])));
        }
        // Pin-scope cap limits — only emitted by the backend when the
        // timing graph has been built (the probe walks Search::endpoints()).
        // Section header notes the gating so the user knows why it might
        // be empty before they've hit the "List endpoints" button.
        if (pinCapLimits.length > 0) {
            this._limScrollArea.appendChild(
                this._makeLimSection(
                    'Pin-Scope Cap Limits  ([get_pins …] — limited to endpoints)',
                    ['Pin', 'Cap max', 'Cap min'],
                    pinCapLimits.map(p => [
                        p.pin || '—',
                        p.cap_max != null ? `${p.cap_max.toPrecision(3)} ${cu}` : '—',
                        p.cap_min != null ? `${p.cap_min.toPrecision(3)} ${cu}` : '—',
                    ])));
        }

        // ── Disabled timing ──────────────────────────────────────────────
        // Sits at the bottom because it's a different kind of constraint
        // (set_disable_timing rather than set_max_*) and tends to be
        // long when present.
        if (disables.length > 0) {
            const scopeLabel = {
                pin:         'pin',
                port:        'port',
                lib_port:    'liberty port',
                cell_port:   'cell',
                inst_port:   'instance',
            };
            // Compose the specifier column from up to four sources:
            //   d.all                    — set_disable_timing without any -from/-to
            //   d.from / d.to (string[]) — set_disable_timing -from X / -to Y (singletons)
            //   d.from_to (object[])     — set_disable_timing -from X -to Y (paired)
            //   d.timing_arc_sets        — DisabledCellPorts arc-set granularity
            // Earlier versions only walked from/to and missed paired -from/-to,
            // which made instance-scope rows appear with just the instance
            // name and an "(empty)" specifier.
            const specifier = (d) => {
                const lines = [];
                if (!('all' in d)) {
                    // pin / port / lib_port scope — no specifier fields.
                } else if (d.all) {
                    lines.push('(all timing)');
                } else {
                    const fromOnly = (d.from || []).join(', ');
                    const toOnly   = (d.to   || []).join(', ');
                    if (fromOnly) lines.push(`-from ${fromOnly}`);
                    if (toOnly)   lines.push(`-to ${toOnly}`);
                    for (const p of (d.from_to || [])) {
                        lines.push(`-from ${p.from} -to ${p.to}`);
                    }
                    if (lines.length === 0) lines.push('(empty)');
                }
                for (const a of (d.timing_arc_sets || [])) {
                    lines.push('• ' + a);
                }
                return lines.join('\n');
            };
            this._limScrollArea.appendChild(
                this._makeLimSection('Disabled Timing  (set_disable_timing)',
                    ['Scope', 'Name', 'Specifier'],
                    disables.map(d => [
                        scopeLabel[d.scope] || d.scope,
                        d.name || '—',
                        specifier(d),
                    ])));
        }
    }

    // Group header used to introduce a cluster of related sections (e.g.
    // the family of set_max_* limit tables). Smaller and visually
    // distinct from the per-section header that _makeLimSection draws.
    _makeLimGroupHeader(title, subtitle) {
        const div = document.createElement('div');
        div.style.cssText =
            'margin:14px 2px 6px 2px;padding:6px 10px;border-radius:4px;' +
            'background:var(--bg-input-deep);' +
            'border-left:3px solid var(--accent-tab);';
        const h = document.createElement('div');
        h.style.cssText =
            'font-size:12px;font-weight:600;color:var(--fg-primary);' +
            'letter-spacing:0.02em;';
        h.textContent = title;
        div.appendChild(h);
        if (subtitle) {
            const s = document.createElement('div');
            s.style.cssText =
                'font-size:11px;color:var(--fg-muted);margin-top:1px;';
            s.textContent = subtitle;
            div.appendChild(s);
        }
        return div;
    }

    // Build a collapsible section with a header and a compact table.
    _makeLimSection(title, headers, rows) {
        const section = document.createElement('div');
        section.style.cssText =
            'margin-bottom:12px;border:1px solid var(--border);border-radius:4px;overflow:hidden;';

        const hdr = document.createElement('div');
        hdr.style.cssText =
            'display:flex;align-items:center;gap:6px;padding:5px 10px;' +
            'background:var(--bg-header);cursor:pointer;user-select:none;';
        const arrow = document.createElement('span');
        arrow.style.cssText = 'font-size:11px;color:var(--fg-muted);transition:transform 0.15s;';
        arrow.textContent = '▼';
        const titleSpan = document.createElement('span');
        titleSpan.style.cssText = 'font-size:12px;font-weight:600;color:var(--fg-primary);';
        titleSpan.textContent = title;
        const countSpan = document.createElement('span');
        countSpan.style.cssText = 'font-size:12px;color:var(--fg-muted);margin-left:auto;';
        countSpan.textContent = `${rows.length} row${rows.length !== 1 ? 's' : ''}`;
        hdr.appendChild(arrow);
        hdr.appendChild(titleSpan);
        hdr.appendChild(countSpan);
        section.appendChild(hdr);

        const body = document.createElement('div');
        body.style.cssText = 'overflow-x:auto;';

        const table = document.createElement('table');
        table.style.cssText =
            'width:100%;border-collapse:collapse;font-size:12px;font-family:monospace;';

        const thead = document.createElement('thead');
        const headerRow = document.createElement('tr');
        headers.forEach(h => {
            const th = document.createElement('th');
            th.style.cssText =
                'padding:3px 8px;text-align:left;font-size:11px;font-weight:600;' +
                'color:var(--fg-muted);border-bottom:1px solid var(--border);' +
                'background:var(--bg-header);white-space:nowrap;';
            th.textContent = h;
            // Surface the SDC vocabulary behind the abbreviation when we
            // recognise the column heading; falls back silently otherwise
            // so headers we haven't documented don't get a misleading tip.
            if (LIMIT_HEADER_TIPS[h]) th.title = LIMIT_HEADER_TIPS[h];
            headerRow.appendChild(th);
        });
        thead.appendChild(headerRow);
        table.appendChild(thead);

        const tbody = document.createElement('tbody');
        rows.forEach((row, i) => {
            const tr = document.createElement('tr');
            tr.style.background = i % 2 === 0 ? 'var(--bg-main)' : 'var(--bg-panel)';
            row.forEach((cell, ci) => {
                const td = document.createElement('td');
                // Cells with embedded newlines (e.g. the disabled-timing
                // specifier column where each -from/-to pair lives on its
                // own line) need pre-line wrapping so the breaks survive
                // the textContent set; everything else stays nowrap so
                // long path names still get the horizontal scroll bar.
                const hasMulti = typeof cell === 'string' && cell.includes('\n');
                td.style.cssText =
                    'padding:3px 8px;color:' + (ci === 0 ? 'var(--fg-primary)' : 'var(--fg-secondary)') +
                    ';white-space:' + (hasMulti ? 'pre-line' : 'nowrap') +
                    ';vertical-align:top;border-bottom:1px solid var(--border-subtle);';
                td.textContent = cell;
                tr.appendChild(td);
            });
            tbody.appendChild(tr);
        });
        table.appendChild(tbody);
        body.appendChild(table);
        section.appendChild(body);

        // Collapse toggle
        let collapsed = false;
        hdr.addEventListener('click', () => {
            collapsed = !collapsed;
            body.style.display = collapsed ? 'none' : '';
            arrow.style.transform = collapsed ? 'rotate(-90deg)' : '';
        });

        return section;
    }

    // ── SVG helpers ──────────────────────────────────────────────────────────

    _makeSvg(w, h) {
        const svg = document.createElementNS(SVG_NS, 'svg');
        svg.setAttribute('width', w);
        svg.setAttribute('height', h);
        svg.setAttribute('viewBox', `0 0 ${w} ${h}`);
        svg.style.display = 'block';
        return svg;
    }

    _svgLine(svg, x1, y1, x2, y2, stroke, width) {
        const line = document.createElementNS(SVG_NS, 'line');
        line.setAttribute('x1', x1);
        line.setAttribute('y1', y1);
        line.setAttribute('x2', x2);
        line.setAttribute('y2', y2);
        line.setAttribute('stroke', stroke);
        line.setAttribute('stroke-width', width);
        svg.appendChild(line);
        return line;
    }

    _svgText(svg, x, y, text, fill, size, anchor) {
        const t = document.createElementNS(SVG_NS, 'text');
        t.setAttribute('x', x);
        t.setAttribute('y', y);
        t.setAttribute('fill', fill);
        t.setAttribute('font-size', size);
        t.setAttribute('text-anchor', anchor || 'start');
        t.textContent = text;
        svg.appendChild(t);
        return t;
    }

    // Draw a HIGH/LOW square-wave clock path from a sorted edge list. Returns
    // the SVG <path> element (already tagged with data-role="clock-waveform"
    // and appended to `svg`). Single source of truth for the four diagrams
    // that previously hand-rolled this with subtle differences.
    //
    //   edges   — sorted [{t, isRise}, ...] within [tStart, tEnd]
    //   tx      — function t → x pixel
    //   highY   — y of the HIGH level
    //   lowY    — y of the LOW level
    //   startY  — clock state just before tStart (HIGH or LOW)
    //   tStart  — left edge of the path in time units
    //   tEnd    — right edge in time units
    //   stroke, strokeWidth — visual; CSS-var strings encouraged
    _drawClockPath(svg, edges, tx, highY, lowY, startY, tStart, tEnd,
                   stroke, strokeWidth) {
        if (!edges.length && tStart === tEnd) return null;
        let d = `M ${tx(tStart)} ${startY} `;
        let prevY = startY;
        for (const { t, isRise } of edges) {
            const x = tx(t);
            const nextY = isRise ? highY : lowY;
            d += `L ${x} ${prevY} L ${x} ${nextY} `;
            prevY = nextY;
        }
        d += `L ${tx(tEnd)} ${prevY}`;
        const path = document.createElementNS(
            SVG_NS, 'path');
        path.setAttribute('d', d);
        path.setAttribute('stroke', stroke);
        path.setAttribute('stroke-width', strokeWidth);
        path.setAttribute('fill', 'none');
        path.dataset.role = 'clock-waveform';
        svg.appendChild(path);
        return path;
    }

    // Stroked rect outline (no fill). Used by every diagram to frame the
    // data bar.
    _svgOutlineRect(svg, x, y, w, h, stroke = 'var(--border)', width = '1') {
        if (w <= 0 || h <= 0) return null;
        const r = document.createElementNS(SVG_NS, 'rect');
        r.setAttribute('x', x);
        r.setAttribute('y', y);
        r.setAttribute('width', w);
        r.setAttribute('height', h);
        r.setAttribute('fill', 'none');
        r.setAttribute('stroke', stroke);
        r.setAttribute('stroke-width', width);
        svg.appendChild(r);
        return r;
    }

    // Filled rect with optional fill-opacity. Single source of truth for
    // the bar-band drawing across the port-delay and endpoint diagrams.
    _svgRect(svg, x, y, w, h, fill, opacity) {
        if (w <= 0 || h <= 0) return null;
        const r = document.createElementNS(SVG_NS, 'rect');
        r.setAttribute('x', x);
        r.setAttribute('y', y);
        r.setAttribute('width', w);
        r.setAttribute('height', h);
        r.style.fill = fill;
        if (opacity != null) r.style.fillOpacity = opacity;
        svg.appendChild(r);
        return r;
    }

    // Filled rect tagged with data-band="<kind>", with the fill resolved from
    // the BAND_COLORS map. Single source of truth for the diagram bar zones
    // — kind drives both the visible color and the test-queryable attribute.
    _svgBandRect(svg, x, y, w, h, kind, opacity) {
        const fill = BAND_COLORS[kind];
        if (!fill) return null;
        const r = this._svgRect(svg, x, y, w, h, fill, opacity);
        if (r) r.dataset.band = kind;
        return r;
    }

    // Build a {min, max, hasSpread} object from a (max, min) pair where
    // either side may be missing (set_input_delay can constrain only one
    // direction). Returns null when both are absent.
    _makeDelaySet(mx, mn) {
        if (mx == null && mn == null) return null;
        const max = mx != null ? mx : mn;
        const min = mn != null ? mn : max;
        return { min, max, hasSpread: min !== max };
    }

    // Attach a native SVG tooltip to any element. Browsers surface <title>
    // children as hover tooltips identical to the HTML `title` attribute.
    _svgTitle(el, text) {
        if (!el || !text) return el;
        const title = document.createElementNS(SVG_NS, 'title');
        title.textContent = text;
        el.appendChild(title);
        return el;
    }

    // Element-agnostic tooltip setter: HTML elements get the `title`
    // attribute, SVG elements get a child <title>. Lets call sites
    // sprinkle tooltips without checking the element kind first.
    _setTooltip(el, text) {
        if (!el || !text) return el;
        if (el.namespaceURI === SVG_NS) {
            this._svgTitle(el, text);
        } else {
            el.title = text;
        }
        return el;
    }
}
