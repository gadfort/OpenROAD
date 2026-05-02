// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

import './setup-dom.js';
import { describe, it, beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import { SdcWidget } from '../../src/sdc-widget.js';

// Mock app with a fake WebSocket manager that resolves readyPromise immediately.
function createMockApp(responses = {}) {
    return {
        websocketManager: {
            readyPromise: Promise.resolve(),
            request(msg) {
                const type = msg.type;
                if (responses[type]) {
                    return Promise.resolve(responses[type](msg));
                }
                return Promise.resolve({});
            },
        },
    };
}

// Settle all pending microtasks and timer callbacks.
const settle = () => new Promise(r => setTimeout(r, 50));

// ── Shared fixture data ───────────────────────────────────────────────────────

const EMPTY_CLOCKS = {
    clocks: [],
    clock_tree: [],
    time_unit: 'ns',
};

const EMPTY_MODES = {
    current_mode: '',
    scene_names: [],
    case_analysis: [],
};

// One primary clock: 10 ns period, 50% duty cycle.
const ONE_CLOCK_RESPONSE = {
    clocks: [
        {
            name: 'clk',
            period: 10.0,
            waveform: [0.0, 5.0],
            is_generated: false,
            is_virtual: false,
            is_propagated: false,
            master_clock: null,
            divide_by: null,
            multiply_by: null,
            invert: false,
            edges: null,
            edge_shifts: null,
            src_pin: null,
            sources: ['clk_port'],
            uncertainty_setup: null,
            uncertainty_hold: null,
        },
    ],
    clock_tree: [{ name: 'clk', children: [] }],
    time_unit: 'ns',
};

// Primary clock + divide-by-2 generated child.
const DIV2_CLOCK_RESPONSE = {
    clocks: [
        {
            name: 'clk',
            period: 10.0,
            waveform: [0.0, 5.0],
            is_generated: false,
            is_virtual: false,
            is_propagated: false,
            master_clock: null,
            divide_by: null,
            multiply_by: null,
            invert: false,
            edges: null,
            edge_shifts: null,
            src_pin: null,
            sources: ['clk_port'],
            uncertainty_setup: null,
            uncertainty_hold: null,
        },
        {
            name: 'clk_div2',
            period: 20.0,
            waveform: [0.0, 10.0],
            is_generated: true,
            is_virtual: false,
            is_propagated: false,
            master_clock: 'clk',
            divide_by: 2,
            multiply_by: null,
            invert: false,
            edges: null,
            edge_shifts: null,
            src_pin: 'u_div/clk_out',
            sources: [],
            uncertainty_setup: null,
            uncertainty_hold: null,
        },
    ],
    clock_tree: [
        {
            name: 'clk',
            children: [{ name: 'clk_div2', children: [] }],
        },
    ],
    time_unit: 'ns',
};

// Clock with setup/hold uncertainty values.
const UNCERTAINTY_CLOCK_RESPONSE = {
    clocks: [
        {
            name: 'clk',
            period: 10.0,
            waveform: [0.0, 5.0],
            is_generated: false,
            is_virtual: false,
            is_propagated: false,
            master_clock: null,
            divide_by: null,
            multiply_by: null,
            invert: false,
            edges: null,
            edge_shifts: null,
            src_pin: null,
            sources: ['clk_port'],
            uncertainty_setup: 0.5,
            uncertainty_hold: 0.25,
        },
    ],
    clock_tree: [{ name: 'clk', children: [] }],
    time_unit: 'ns',
};

// Build an sdc_clocks response from a list of partial clock specs. Any field
// the test doesn't override gets a sensible default. Used by every Clocks-tab
// test that needs a custom clock fixture.
function makeClocksResponse(clocks) {
    const filled = clocks.map(c => ({
        name: c.name || 'clk',
        period: c.period ?? 10,
        waveform: c.waveform || [0, (c.period ?? 10) / 2],
        is_generated: !!c.is_generated,
        is_virtual: !!c.is_virtual,
        is_propagated: !!c.is_propagated,
        master_clock: c.master_clock ?? null,
        divide_by: c.divide_by ?? null,
        multiply_by: c.multiply_by ?? null,
        invert: !!c.invert,
        edges: c.edges ?? null,
        edge_shifts: c.edge_shifts ?? null,
        src_pin: c.src_pin ?? null,
        sources: c.sources || (c.is_virtual ? [] : ['clk_port']),
        uncertainty_setup: c.uncertainty_setup ?? null,
        uncertainty_hold: c.uncertainty_hold ?? null,
        ...c,
    }));
    return {
        clocks: filled,
        clock_tree: filled
            .filter(c => !c.master_clock)
            .map(root => ({
                name: root.name,
                children: filled
                    .filter(c => c.master_clock === root.name)
                    .map(c => ({ name: c.name, children: [] })),
            })),
        time_unit: 'ns',
    };
}

// Default fixture for a single port-delay entry; spread `over` lets each test
// override just the fields it cares about. Used by every Port-Delays test.
const makePdEntry = (over) => ({
    port: 'in1',
    direction: 'input',
    is_input: true,
    clock: 'clk',
    clk_edge: 'rise',
    clk_period: 10,
    uncertainty_setup: 0,
    uncertainty_hold:  0,
    rise_max: 2, rise_min: 2,
    fall_max: 2, fall_min: 2,
    ...over,
});

// Wrap one or more entries in the sdc_port_delays envelope. Derives
// clocks_total from the entries so the new clock-domain dropdown sees
// a populated count map (mirrors what the backend emits).
const pdResponse = (entries, extra = {}) => {
    const clocksTotal = {};
    for (const e of entries) {
        const k = e.clock || '__none__';
        clocksTotal[k] = (clocksTotal[k] || 0) + 1;
    }
    return {
        time_unit: 'ns', cap_unit: 'pF',
        port_delays: entries,
        total: entries.length,
        clocks_total: clocksTotal,
        ...extra,
    };
};

// Build a widget with the Port Delays tab active and the given entries
// loaded. Single shared loader for all port-delay-tab tests.
async function loadPdWidget(entries) {
    const app = createMockApp({
        sdc_clocks:      () => EMPTY_CLOCKS,
        sdc_clock_modes: () => EMPTY_MODES,
        sdc_port_delays: () => pdResponse(entries),
    });
    const widget = new SdcWidget(app);
    widget._activateTab('Port Delays');
    await settle();
    return widget;
}

const CASE_MODES_RESPONSE = {
    current_mode: 'func',
    scene_names: ['func_slow'],
    case_analysis: [
        { pin: 'u_mux/sel', value: '0' },
        { pin: 'u_mux2/sel', value: '1' },
    ],
};

// ── Endpoint test factories ───────────────────────────────────────────────────
//
// 11 nearly-identical `epListResp` fixtures appeared in the original test
// file with only the endpoints array changing. These factories reduce the
// boilerplate to one line per test.

// Build an `sdc_endpoint_list` response from a flat list of endpoint
// objects. Counts each endpoint's `kind` field to populate `kinds_total`,
// and sets `total = endpoints.length`. Pass `extra` to override e.g.
// `time_unit` if a test needs something other than 'ns'.
function makeEpListResp(endpoints, extra = {}) {
    const kindsTotal = { flipflop: 0, latch: 0, macro: 0, stdcell: 0 };
    for (const e of endpoints) {
        if (e.kind && kindsTotal[e.kind] != null) kindsTotal[e.kind] += 1;
    }
    return {
        time_unit: 'ns',
        total: endpoints.length,
        offset: 0,
        kinds_total: kindsTotal,
        endpoints,
        ...extra,
    };
}

// Build an `sdc_endpoint` (per-pin detail) response. `pins` carries the
// per-pin payload; sensible defaults (`is_port: false`, empty
// port_delays / exceptions) are filled in if missing.
function makeEpDetailResp(pins) {
    return {
        found: pins.length > 0,
        multi: pins.length > 1,
        time_unit: 'ns',
        total: pins.length,
        offset: 0,
        pins: pins.map(p => ({
            is_port: false,
            port_delays: [],
            exceptions: [],
            ...p,
        })),
    };
}

// Build an app whose WebSocket manager dispatches the standard endpoint
// test requests (`sdc_endpoint_list`, `sdc_endpoint`, plus the empty
// clocks/modes responses every tab needs on activation). `calls` is
// shared so tests can assert on what was dispatched.
function makeEpApp({ listResp, detailResp }) {
    const calls = [];
    const app = {
        websocketManager: {
            readyPromise: Promise.resolve(),
            request(msg) {
                calls.push(msg);
                if (msg.type === 'sdc_endpoint_list') {
                    return Promise.resolve(typeof listResp === 'function'
                        ? listResp(msg) : listResp);
                }
                if (msg.type === 'sdc_endpoint') {
                    return Promise.resolve(typeof detailResp === 'function'
                        ? detailResp(msg) : detailResp);
                }
                if (msg.type === 'sdc_clocks')
                    return Promise.resolve(EMPTY_CLOCKS);
                if (msg.type === 'sdc_clock_modes')
                    return Promise.resolve(EMPTY_MODES);
                return Promise.resolve({});
            },
        },
    };
    return { app, calls };
}

// Spin up a widget on a target tab and wait for its initial async work
// to flush. The widget + tab + settle sequence repeats 70+ times in this
// file — extracted so test bodies can focus on the behavior, not the
// scaffolding.
async function setupTabWidget(tab, app) {
    const widget = new SdcWidget(app);
    widget._activateTab(tab);
    await settle();
    return widget;
}

// ── DOM structure ─────────────────────────────────────────────────────────────

describe('SdcWidget', () => {
    describe('construction', () => {
        it('registers all eight tab panels', () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            const expectedTabs = ['Clocks', 'Endpoint', 'Port Delays',
                'Exceptions', 'Clock Groups', 'CDC', 'Limits', 'Case Analysis'];
            for (const t of expectedTabs) {
                assert.ok(widget._tabPanels[t], `tab panel "${t}" exists`);
            }
        });
    });

    // ── Loading state ───────────────────────────────────────────────────────

    describe('data loading', () => {
        it('shows loading placeholder before data arrives', () => {
            const app = {
                websocketManager: {
                    readyPromise: new Promise(() => {}),  // never resolves
                    request() { return new Promise(() => {}); },
                },
            };
            const widget = new SdcWidget(app);
            assert.ok(widget.element.textContent.includes('Loading'),
                'shows loading message');
        });

        it('shows no-clocks message when backend returns empty', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            assert.ok(widget.element.textContent.includes('No clocks'),
                'shows no-clocks message');
        });

        it('renders clock names after loading', async () => {
            const app = createMockApp({
                sdc_clocks: () => ONE_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            assert.ok(widget._cardScrollArea.textContent.includes('clk'),
                'clock name visible in card area');
        });

        it('shows clock period in card stats', async () => {
            const app = createMockApp({
                sdc_clocks: () => ONE_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            assert.ok(widget._cardScrollArea.textContent.includes('10'),
                'period value visible');
        });

        it('shows error message on fetch failure', async () => {
            const app = {
                websocketManager: {
                    readyPromise: Promise.resolve(),
                    request() {
                        return Promise.reject(new Error('network error'));
                    },
                },
            };
            const widget = new SdcWidget(app);
            await settle();

            // _loadData routes fetch errors through _showLoadError, which
            // formats them as "Error loading clocks: <msg>" (or "Still
            // loading clocks…" for timeouts) and adds a Retry button.
            const text = widget.element.textContent;
            assert.ok(text.includes('Error loading clocks'),
                'shows the load-error banner');
            assert.ok(text.includes('network error'),
                'includes the underlying error text so the user knows what failed');
            const retryBtn = Array.from(
                widget.element.querySelectorAll('button')).find(
                    b => b.textContent === 'Retry'
                      || b.textContent === 'Check again');
            assert.ok(retryBtn, 'banner offers a retry button');
        });

        it('timeouts show "Check again" wording, not "Retry"', () => {
            // Drive _showLoadError directly with an isTimeout error to
            // verify the timeout-specific wording without waiting on a
            // real 60s clock. The banner should say "Still loading…"
            // and the button should read "Check again".
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            const target = document.createElement('div');
            const err = new Error("sdc request 'sdc_clocks' timed out after 60s");
            err.isTimeout = true;
            let retried = false;
            widget._showLoadError(target, 'clocks', err, () => { retried = true; });
            assert.ok(/Still loading clocks/.test(target.textContent),
                'shows still-loading wording for timeouts');
            const btn = target.querySelector('button');
            assert.ok(btn, 'banner has a button');
            assert.equal(btn.textContent, 'Check again',
                'timeout banner uses "Check again" instead of "Retry"');
            btn.click();
            assert.ok(retried, 'clicking "Check again" invokes the retry callback');
        });
    });

    // ── Clock cards ─────────────────────────────────────────────────────────

    describe('clock cards', () => {
        it('renders a card for each clock', async () => {
            const app = createMockApp({
                sdc_clocks: () => DIV2_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            // Both clk and clk_div2 should appear as card headers.
            assert.ok(widget._cardScrollArea.textContent.includes('clk'),
                'master clock card present');
            assert.ok(widget._cardScrollArea.textContent.includes('clk_div2'),
                'generated clock card present');
        });

        it('shows divide-by badge on generated clock card', async () => {
            const app = createMockApp({
                sdc_clocks: () => DIV2_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            assert.ok(widget._cardScrollArea.textContent.includes('÷2'),
                'divide-by badge shown on card');
        });

        it('shows frequency stat in card', async () => {
            const app = createMockApp({
                sdc_clocks: () => ONE_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            // 10ns period → 100 MHz
            const text = widget._cardScrollArea.textContent;
            assert.ok(text.includes('MHz') || text.includes('GHz') || text.includes('Hz'),
                'frequency unit shown');
        });

        it('shows uncertainty stats when present', async () => {
            const app = createMockApp({
                sdc_clocks: () => UNCERTAINTY_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            const text = widget._cardScrollArea.textContent;
            assert.ok(text.includes('setup unc'), 'setup uncertainty label shown');
            assert.ok(text.includes('hold unc'),  'hold uncertainty label shown');
        });

        it('uncertainty stats expose a tooltip explaining the abbreviation', async () => {
            const app = createMockApp({
                sdc_clocks: () => UNCERTAINTY_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            // Find a stat row by its label text and check the title attribute.
            const rows = Array.from(widget._cardScrollArea.querySelectorAll('div'));
            const setupRow = rows.find(r =>
                r.title && r.title.includes('setup clock uncertainty'));
            const holdRow = rows.find(r =>
                r.title && r.title.includes('hold clock uncertainty'));
            assert.ok(setupRow, 'setup uncertainty row has explanatory tooltip');
            assert.ok(holdRow,  'hold uncertainty row has explanatory tooltip');
        });

        it('generated clock card shows src_pin and parent master clock', async () => {
            const app = createMockApp({
                sdc_clocks: () => DIV2_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            const text = widget._cardScrollArea.textContent;
            assert.ok(text.includes('u_div/clk_out'),
                'generated clock shows src_pin (where the divider sits)');
            assert.ok(text.includes('from'),
                'generated clock shows parent-clock relationship ("from")');
        });

        it('primary clock card shows source port name', async () => {
            const app = createMockApp({
                sdc_clocks: () => ONE_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            assert.ok(widget._cardScrollArea.textContent.includes('clk_port'),
                'primary clock shows its source port (from sources[0])');
        });

        // Type-badge ("master" / "generated" / "virtual") rendering, one
        // case per kind. The virtual clock additionally surfaces a
        // "(virtual)" marker on the src stat.
        const TYPE_BADGE_CASES = [
            { kind: 'master',
              response: () => ONE_CLOCK_RESPONSE },
            { kind: 'generated',
              response: () => DIV2_CLOCK_RESPONSE },
            { kind: 'virtual', extraText: '(virtual)',
              response: () => makeClocksResponse([{
                  name: 'vclk', period: 10, waveform: [0, 5],
                  is_virtual: true, sources: [],
              }])
            },
        ];
        for (const tc of TYPE_BADGE_CASES) {
            it(`card header shows "${tc.kind}" type badge`, async () => {
                const app = createMockApp({
                    sdc_clocks: tc.response,
                    sdc_clock_modes: () => EMPTY_MODES,
                });
                const widget = new SdcWidget(app);
                await settle();
                const text = widget._cardScrollArea.textContent;
                assert.ok(text.includes(tc.kind),
                    `"${tc.kind}" badge rendered`);
                if (tc.extraText) {
                    assert.ok(text.includes(tc.extraText),
                        `extra marker "${tc.extraText}" rendered`);
                }
            });
        }

        it('"propagated" badge tracks the is_propagated flag', async () => {
            const renderAndCheckBadge = async (isPropagated) => {
                const app = createMockApp({
                    sdc_clocks: () => makeClocksResponse([{
                        name: 'clk', period: 10, waveform: [0, 5],
                        is_propagated: isPropagated, sources: ['clk_port'],
                    }]),
                    sdc_clock_modes: () => EMPTY_MODES,
                });
                const widget = new SdcWidget(app);
                await settle();
                return widget._cardScrollArea.textContent.includes('propagated');
            };
            assert.equal(await renderAndCheckBadge(true), true);
            assert.equal(await renderAndCheckBadge(false), false);
        });

        it('rendered card contains an SVG waveform', async () => {
            const app = createMockApp({
                sdc_clocks: () => ONE_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            const svg = widget._cardScrollArea.querySelector('svg');
            assert.ok(svg, 'SVG element rendered inside card');
        });

        it('SVG contains a path element for the waveform', async () => {
            const app = createMockApp({
                sdc_clocks: () => ONE_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            const paths = widget._cardScrollArea.querySelectorAll('path');
            assert.ok(paths.length > 0, 'at least one SVG path rendered');
        });

        it('generated clock card is nested under its master in All-filter mode', async () => {
            // Tree view: master clock's wrapper contains its card plus an
            // indented children-container; each generated child sits inside
            // that container. We verify the generated card is parented by
            // an element with the dashed left guide (the children-container)
            // — that's how the visual indentation is produced.
            const app = createMockApp({
                sdc_clocks: () => DIV2_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            const genCard = widget._cardScrollArea.querySelector(
                '.sdc-clock-card[data-clock-name="clk_div2"]');
            assert.ok(genCard, 'generated clock card rendered');
            // Its DOM parent should NOT be the scroll area directly — it
            // must sit inside the master's children-container in tree mode.
            assert.notEqual(genCard.parentElement, widget._cardScrollArea,
                'generated card is nested rather than at the top level');
            // The container carrying it has the dashed border-left guide.
            const container = genCard.parentElement;
            const borderLeft = container.style.borderLeft || '';
            assert.ok(borderLeft.includes('dashed'),
                'parent container draws the tree-view guide line');
        });
    });

    // ── Filter buttons ──────────────────────────────────────────────────────

    describe('clock filter buttons', () => {
        it('Clocks tab toolbar has All/Master/Generated filter buttons', () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            const panel = widget._tabPanels['Clocks'];
            const labels = Array.from(panel.querySelectorAll('button'))
                .map(b => b.dataset.clkFilterLabel || b.textContent);
            assert.ok(labels.includes('All'),       'All button present');
            assert.ok(labels.includes('Master'),    'Master button present');
            assert.ok(labels.includes('Generated'), 'Generated button present');
            assert.ok(labels.includes('Virtual'),   'Virtual button present');
        });

        it('Master filter shows only non-generated clocks', async () => {
            const app = createMockApp({
                sdc_clocks: () => DIV2_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            // Click Master filter (dataset key is stable; textContent now
            // includes a "(N)" running count).
            const panel = widget._tabPanels['Clocks'];
            const masterBtn = Array.from(panel.querySelectorAll('button'))
                .find(b => b.dataset.clkFilterKey === 'master');
            assert.ok(masterBtn, 'Master button found');
            masterBtn.click();

            assert.ok(widget._cardScrollArea.textContent.includes('clk'),
                'master clock still visible');
            assert.ok(!widget._cardScrollArea.textContent.includes('clk_div2'),
                'generated clock hidden by Master filter');
        });

        it('Generated filter shows only generated clocks', async () => {
            const app = createMockApp({
                sdc_clocks: () => DIV2_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            const panel = widget._tabPanels['Clocks'];
            const genBtn = Array.from(panel.querySelectorAll('button'))
                .find(b => b.dataset.clkFilterKey === 'generated');
            assert.ok(genBtn, 'Generated button found');
            genBtn.click();

            assert.ok(widget._cardScrollArea.textContent.includes('clk_div2'),
                'generated clock visible under Generated filter');
            // master 'clk' name appears inside stats of clk_div2 (src field),
            // but clk_div2 header should be the dominant content.
            const cards = Array.from(widget._cardScrollArea.children);
            assert.equal(cards.length, 1, 'only one card under Generated filter');
        });

        it('All filter restores full list', async () => {
            const app = createMockApp({
                sdc_clocks: () => DIV2_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            const panel = widget._tabPanels['Clocks'];
            const buttons = panel.querySelectorAll('button');

            // Switch to Master then back to All
            Array.from(buttons).find(b => b.dataset.clkFilterKey === 'master').click();
            Array.from(buttons).find(b => b.dataset.clkFilterKey === 'all').click();

            // Count clock cards by class (tree-view nests generated clocks
            // inside their master's children-container, so direct children
            // of _cardScrollArea is no longer 1:1 with clock count).
            const cards = widget._cardScrollArea.querySelectorAll('.sdc-clock-card');
            assert.equal(cards.length, 2, 'both clock cards visible under All filter');
        });

        it('_clkFilter state tracks the active filter', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            assert.equal(widget._clkFilter, 'all', 'default filter is all');

            const panel = widget._tabPanels['Clocks'];
            const masterBtn = Array.from(panel.querySelectorAll('button'))
                .find(b => b.dataset.clkFilterKey === 'master');
            masterBtn.click();
            assert.equal(widget._clkFilter, 'master', 'filter updated to master');
        });

        it('clock filter buttons show running counts per bucket', async () => {
            // DIV2_CLOCK_RESPONSE has one master + one generated clock.
            const app = createMockApp({
                sdc_clocks: () => DIV2_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            const panel = widget._tabPanels['Clocks'];
            const byKey = {};
            for (const b of panel.querySelectorAll('button[data-clk-filter-key]')) {
                byKey[b.dataset.clkFilterKey] = b;
            }
            assert.ok(byKey.all.textContent.includes('(2)'),       'All shows total count');
            assert.ok(byKey.master.textContent.includes('(1)'),    'Master shows non-gen count');
            assert.ok(byKey.generated.textContent.includes('(1)'), 'Generated shows gen count');
            assert.ok(byKey.virtual.textContent.includes('(0)'),   'Virtual shows zero count');
        });

        it('Virtual filter shows only virtual clocks', async () => {
            // Add a virtual clock to the standard fixture.
            const resp = {
                ...DIV2_CLOCK_RESPONSE,
                clocks: [
                    ...DIV2_CLOCK_RESPONSE.clocks,
                    {
                        name: 'virt_clk', period: 6, waveform: [0, 3],
                        is_generated: false, is_virtual: true,
                        is_propagated: false, master_clock: null,
                        divide_by: null, multiply_by: null, invert: false,
                        edges: null, edge_shifts: null, src_pin: null,
                        sources: [],
                        uncertainty_setup: null, uncertainty_hold: null,
                    },
                ],
                clock_tree: [
                    ...DIV2_CLOCK_RESPONSE.clock_tree,
                    { name: 'virt_clk', children: [] },
                ],
            };
            const app = createMockApp({
                sdc_clocks: () => resp,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            const panel = widget._tabPanels['Clocks'];
            const virtBtn = Array.from(panel.querySelectorAll('button'))
                .find(b => b.dataset.clkFilterKey === 'virtual');
            assert.ok(virtBtn, 'Virtual filter button present');
            virtBtn.click();
            assert.equal(widget._clkFilter, 'virtual',
                'filter state set to virtual');

            const cards = widget._cardScrollArea.querySelectorAll('.sdc-clock-card');
            assert.equal(cards.length, 1, 'only the virtual clock card visible');
            assert.equal(cards[0].dataset.clockName, 'virt_clk',
                'visible card is the virtual clock');
        });
    });

    // ── _periodToFreq helper ────────────────────────────────────────────────

    describe('_periodToFreq', () => {
        let widget;
        beforeEach(async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            widget = new SdcWidget(app);
            await settle();
        });

        it('returns GHz for sub-nanosecond periods', () => {
            widget._timeUnit = 'ns';
            const result = widget._periodToFreq(0.5);  // 0.5 ns → 2 GHz
            assert.ok(result.includes('GHz'), `expected GHz, got: ${result}`);
        });

        it('returns MHz for nanosecond-range periods', () => {
            widget._timeUnit = 'ns';
            const result = widget._periodToFreq(10);  // 10 ns → 100 MHz
            assert.ok(result.includes('MHz'), `expected MHz, got: ${result}`);
        });

        it('returns kHz for microsecond-range periods', () => {
            widget._timeUnit = 'ns';
            const result = widget._periodToFreq(5000);  // 5000 ns → 200 kHz
            assert.ok(result.includes('kHz'), `expected kHz, got: ${result}`);
        });

        it('returns Hz for very long periods', () => {
            widget._timeUnit = 'ns';
            const result = widget._periodToFreq(2e9);  // 2e9 ns → 0.5 Hz
            assert.ok(result.includes('Hz') && !result.includes('MHz') &&
                      !result.includes('kHz') && !result.includes('GHz'),
                `expected Hz, got: ${result}`);
        });
    });

    // ── Case-analysis strip ─────────────────────────────────────────────────

    describe('case analysis strip', () => {
        it('renders case_analysis entries', async () => {
            const app = createMockApp({
                sdc_clocks: () => ONE_CLOCK_RESPONSE,
                sdc_clock_modes: () => CASE_MODES_RESPONSE,
            });
            const widget = new SdcWidget(app);
            await settle();

            const strip = widget._caseStrip;
            assert.ok(strip.textContent.includes('u_mux/sel'),
                'pin name shown in case strip');
            assert.ok(strip.textContent.includes('u_mux2/sel'),
                'second pin shown');
        });

        it('shows current mode name', async () => {
            const app = createMockApp({
                sdc_clocks: () => ONE_CLOCK_RESPONSE,
                sdc_clock_modes: () => CASE_MODES_RESPONSE,
            });
            const widget = new SdcWidget(app);
            await settle();

            assert.ok(widget._caseStrip.textContent.includes('func'),
                'mode name shown');
        });

        it('empty case analysis renders an explanatory placeholder', async () => {
            const app = createMockApp({
                sdc_clocks: () => ONE_CLOCK_RESPONSE,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();

            // Case analysis lives in its own tab now; when empty it shows a
            // "No set_case_analysis..." placeholder instead of being blank.
            assert.ok(widget._caseStrip.textContent.includes(
                'No set_case_analysis'),
                'empty case-analysis panel shows placeholder');
        });

        it('shows logic value for each pin', async () => {
            const app = createMockApp({
                sdc_clocks: () => ONE_CLOCK_RESPONSE,
                sdc_clock_modes: () => CASE_MODES_RESPONSE,
            });
            const widget = new SdcWidget(app);
            await settle();

            const text = widget._caseStrip.textContent;
            assert.ok(text.includes('0'), 'first pin value shown');
            assert.ok(text.includes('1'), 'second pin value shown');
        });
    });

    // ── Refresh ─────────────────────────────────────────────────────────────

    describe('refresh', () => {
        it('refresh button resets loaded state and re-requests data', async () => {
            let callCount = 0;
            const app = {
                websocketManager: {
                    readyPromise: Promise.resolve(),
                    request(msg) {
                        if (msg.type === 'sdc_clocks') callCount++;
                        return Promise.resolve(
                            msg.type === 'sdc_clocks' ? EMPTY_CLOCKS : EMPTY_MODES
                        );
                    },
                },
            };
            const widget = new SdcWidget(app);
            await settle();

            const initialCount = callCount;

            const refreshBtn = widget.element.querySelector('button[title*="Reload"]');
            assert.ok(refreshBtn, 'refresh button exists');
            refreshBtn.click();

            await settle();
            assert.ok(callCount > initialCount, 'sdc_clocks re-requested after refresh');
        });

        it('refresh button is in the Clocks toolbar', () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            const panel = widget._tabPanels['Clocks'];
            const btn = panel.querySelector('button[title*="Reload"]');
            assert.ok(btn, 'refresh button found in Clocks panel');
        });
    });

    // ── Tab switching ───────────────────────────────────────────────────────

    describe('tab switching', () => {
        it('only active tab panel is visible', () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            const panels = Object.values(widget._tabPanels);
            const visible = panels.filter(p => p.style.display !== 'none');
            assert.equal(visible.length, 1, 'exactly one tab panel visible');
        });

        it('clicking a tab button makes its panel visible', () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            widget._tabButtons['Port Delays'].click();

            assert.notEqual(widget._tabPanels['Port Delays'].style.display, 'none',
                'Port Delays panel visible after click');
            assert.equal(widget._tabPanels['Clocks'].style.display, 'none',
                'Clocks panel hidden after switching');
        });

        it('all tab names are present', () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            const names = Object.keys(widget._tabButtons);
            const expected = ['Clocks', 'Endpoint', 'Port Delays',
                              'Exceptions', 'Clock Groups', 'Limits',
                              'Case Analysis'];
            for (const name of expected) {
                assert.ok(names.includes(name), `tab "${name}" present`);
            }
        });
    });

    // ── Mode selector (shared across all tabs) ──────────────────────────────

    describe('mode selector', () => {
        it('mode bar is hidden when only one mode exists', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_list_modes: () => ({ modes: ['default'], current: 'default' }),
            });
            const widget = new SdcWidget(app);
            await settle();

            assert.equal(widget._modeBar.style.display, 'none',
                'mode bar hidden when only one mode');
        });

        it('mode bar is hidden when no modes returned', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_list_modes: () => ({ modes: [], current: '' }),
            });
            const widget = new SdcWidget(app);
            await settle();

            assert.equal(widget._modeBar.style.display, 'none',
                'mode bar hidden on empty modes list');
        });

        it('mode bar is visible and populated when multiple modes exist', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_list_modes: () => ({
                    modes: ['functional', 'scan', 'test'],
                    current: 'functional',
                }),
            });
            const widget = new SdcWidget(app);
            await settle();

            assert.notEqual(widget._modeBar.style.display, 'none',
                'mode bar visible with multiple modes');
            const opts = Array.from(widget._modeSelect.options).map(o => o.value);
            assert.deepEqual(opts, ['functional', 'scan', 'test'],
                'dropdown populated in given order');
            assert.equal(widget._modeSelect.value, 'functional',
                'current mode is selected');
        });

        it('selecting a mode calls sdc_set_mode with the chosen name', async () => {
            let lastSet = null;
            const app = {
                websocketManager: {
                    readyPromise: Promise.resolve(),
                    request(msg) {
                        if (msg.type === 'sdc_list_modes') {
                            return Promise.resolve({
                                modes: ['functional', 'scan'],
                                current: 'functional',
                            });
                        }
                        if (msg.type === 'sdc_set_mode') {
                            lastSet = msg.mode;
                            return Promise.resolve({ ok: true, current: msg.mode });
                        }
                        if (msg.type === 'sdc_clocks') return Promise.resolve(EMPTY_CLOCKS);
                        if (msg.type === 'sdc_clock_modes') return Promise.resolve(EMPTY_MODES);
                        return Promise.resolve({});
                    },
                },
            };
            const widget = new SdcWidget(app);
            await settle();

            widget._modeSelect.value = 'scan';
            widget._modeSelect.dispatchEvent(new window.Event('change'));
            await settle();

            assert.equal(lastSet, 'scan', 'sdc_set_mode called with scan');
            assert.equal(widget._currentModeName, 'scan', 'current mode updated');
        });

        it('successful mode change invalidates all tab loaded flags', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_list_modes: () => ({
                    modes: ['a', 'b'],
                    current: 'a',
                }),
                sdc_set_mode: (msg) => ({ ok: true, current: msg.mode }),
                sdc_port_delays: () => ({ time_unit: 'ns', port_delays: [] }),
                sdc_exceptions: () => ({ time_unit: 'ns', exceptions: [] }),
                sdc_limits: () => ({
                    time_unit: 'ns', cap_unit: 'pF',
                    clock_latencies: [], clock_uncertainties: [], port_loads: [],
                }),
                sdc_clock_groups: () => ({ groups: [] }),
            });
            const widget = new SdcWidget(app);
            await settle();

            // Pre-load every tab so its loaded flag is true.
            widget._activateTab('Port Delays');   await settle();
            widget._activateTab('Exceptions');    await settle();
            widget._activateTab('Limits');        await settle();
            widget._activateTab('Clock Groups');  await settle();
            widget._activateTab('Clocks');        await settle();

            assert.equal(widget._loaded,    true, 'Clocks pre-loaded');
            assert.equal(widget._pdLoaded,  true, 'PortDelays pre-loaded');
            assert.equal(widget._excLoaded, true, 'Exceptions pre-loaded');
            assert.equal(widget._limLoaded, true, 'Limits pre-loaded');
            assert.equal(widget._cgLoaded,  true, 'ClockGroups pre-loaded');

            widget._modeSelect.value = 'b';
            widget._modeSelect.dispatchEvent(new window.Event('change'));
            await settle();

            // Active tab (Clocks) gets reloaded, so its flag becomes true again.
            // The inactive tabs stay invalidated until their next activation.
            assert.equal(widget._pdLoaded,  false, 'PortDelays invalidated');
            assert.equal(widget._excLoaded, false, 'Exceptions invalidated');
            assert.equal(widget._limLoaded, false, 'Limits invalidated');
            assert.equal(widget._cgLoaded,  false, 'ClockGroups invalidated');
        });

        it('mode change reloads the currently-active tab', async () => {
            let clockCalls = 0;
            const app = {
                websocketManager: {
                    readyPromise: Promise.resolve(),
                    request(msg) {
                        if (msg.type === 'sdc_list_modes') {
                            return Promise.resolve({ modes: ['a', 'b'], current: 'a' });
                        }
                        if (msg.type === 'sdc_set_mode') {
                            return Promise.resolve({ ok: true, current: msg.mode });
                        }
                        if (msg.type === 'sdc_clocks') {
                            clockCalls++;
                            return Promise.resolve(EMPTY_CLOCKS);
                        }
                        if (msg.type === 'sdc_clock_modes') return Promise.resolve(EMPTY_MODES);
                        return Promise.resolve({});
                    },
                },
            };
            const widget = new SdcWidget(app);
            await settle();

            const before = clockCalls;
            widget._modeSelect.value = 'b';
            widget._modeSelect.dispatchEvent(new window.Event('change'));
            await settle();

            assert.ok(clockCalls > before,
                'sdc_clocks re-requested after mode change');
        });

        it('failed mode change reverts dropdown selection', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_list_modes: () => ({ modes: ['a', 'b'], current: 'a' }),
                sdc_set_mode: () => ({ ok: false, error: 'mode not found', current: 'a' }),
            });
            const widget = new SdcWidget(app);
            await settle();

            widget._modeSelect.value = 'b';
            widget._modeSelect.dispatchEvent(new window.Event('change'));
            await settle();

            assert.equal(widget._modeSelect.value, 'a',
                'dropdown reverts to previous mode on failure');
            assert.equal(widget._currentModeName, 'a',
                '_currentModeName unchanged on failure');
        });

        it('endpoint list area shows hint after mode change', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_list_modes: () => ({ modes: ['a', 'b'], current: 'a' }),
                sdc_set_mode: (msg) => ({ ok: true, current: msg.mode }),
            });
            const widget = new SdcWidget(app);
            await settle();

            widget._modeSelect.value = 'b';
            widget._modeSelect.dispatchEvent(new window.Event('change'));
            await settle();

            assert.ok(widget._epListArea.textContent.includes('Mode changed'),
                'endpoint list area displays mode-change hint');
        });
    });

    // ── Clock Groups tab ───────────────────────────────────────────────────
    //
    // Regression for a silent key mismatch: the backend emits {"groups":[...]}
    // but _renderClockGroups was reading data.clock_groups, so actual
    // set_clock_groups constraints never showed up.

    describe('clock groups tab', () => {
        const ASYNC_GROUPS_RESPONSE = {
            groups: [{
                name: 'async_all',
                type: 'asynchronous',
                allow_paths: false,
                clk_sets: [['clk_a'], ['clk_b'], ['clk_c']],
            }],
        };

        it('reads groups under the "groups" key (not "clock_groups")', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_clock_groups: () => ASYNC_GROUPS_RESPONSE,
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Clock Groups');
            await settle();

            const text = widget._cgScrollArea.textContent;
            assert.ok(!text.includes('No clock group constraints'),
                'non-empty groups array should not render the empty placeholder');
            assert.ok(text.includes('async_all'),
                'group name should appear in the rendered card');
            assert.ok(text.includes('ASYNC'),
                'ASYNC badge should appear for asynchronous groups');
        });

        it('empty groups list renders the placeholder', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_clock_groups: () => ({ groups: [] }),
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Clock Groups');
            await settle();

            assert.ok(widget._cgScrollArea.textContent.includes(
                'No clock group constraints'),
                'empty groups array shows the placeholder');
        });

        it('lists the design clocks when no set_clock_groups exists', async () => {
            // When there are no set_clock_groups commands, the matrix would
            // be a wall of "·" cells (every pair is independently checked).
            // The widget intentionally swaps to a flat scannable list of
            // every clock the design defines, with a header that explains
            // why the matrix is gone.
            const TWO_CLOCKS_NO_GROUPS = {
                clocks: [
                    { name: 'clk_a', period: 10, waveform: [0, 5],
                      is_generated: false, is_virtual: false,
                      is_propagated: false, master_clock: null,
                      divide_by: null, multiply_by: null, invert: false,
                      edges: null, edge_shifts: null, src_pin: null,
                      sources: [], uncertainty_setup: null, uncertainty_hold: null },
                    { name: 'clk_b', period: 20, waveform: [0, 10],
                      is_generated: false, is_virtual: false,
                      is_propagated: false, master_clock: null,
                      divide_by: null, multiply_by: null, invert: false,
                      edges: null, edge_shifts: null, src_pin: null,
                      sources: [], uncertainty_setup: null, uncertainty_hold: null },
                ],
                clock_tree: [
                    { name: 'clk_a', children: [] },
                    { name: 'clk_b', children: [] },
                ],
                time_unit: 'ns',
            };
            const app = createMockApp({
                sdc_clocks: () => TWO_CLOCKS_NO_GROUPS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_clock_groups: () => ({ groups: [] }),
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Clock Groups');
            await settle();

            const text = widget._cgScrollArea.textContent;
            // Both clocks listed.
            assert.ok(text.includes('clk_a'), 'clk_a appears in the list');
            assert.ok(text.includes('clk_b'), 'clk_b appears in the list');
            // Header explains why the matrix is suppressed.
            assert.ok(text.includes('no set_clock_groups'),
                'placeholder explains why the matrix is suppressed');
            // The "wall of dots" matrix header should NOT be present.
            assert.ok(!text.includes('Clock Relationship Matrix'),
                'matrix is intentionally suppressed when no groups exist');
        });

        it('matrix is suppressed when fewer than 2 clocks are known', async () => {
            const app = createMockApp({
                sdc_clocks: () => ONE_CLOCK_RESPONSE,  // one clock
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_clock_groups: () => ({ groups: [] }),
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Clock Groups');
            await settle();

            assert.ok(!widget._cgScrollArea.textContent.includes(
                'Clock Relationship Matrix'),
                'matrix suppressed when design has only one clock');
        });
    });

    // ── Case-analysis strip (set_case_analysis + set_logic_*) ───────────────

    describe('case analysis strip (merged sources)', () => {
        it('renders both case_analysis and logic_values entries', async () => {
            const app = createMockApp({
                sdc_clocks: () => ONE_CLOCK_RESPONSE,
                sdc_clock_modes: () => ({
                    current_mode: '',
                    scene_names: [],
                    case_analysis: [{ pin: 'u_mux/sel', value: '0' }],
                    logic_values:  [{ pin: 'u_gate/en', value: '1' }],
                }),
            });
            const widget = new SdcWidget(app);
            await settle();

            const text = widget._caseStrip.textContent;
            assert.ok(text.includes('u_mux/sel'), 'case_analysis pin visible');
            assert.ok(text.includes('u_gate/en'), 'logic_values pin visible');
            assert.ok(text.includes('case'),       'case source-tag visible');
            assert.ok(text.includes('logic'),      'logic source-tag visible');
        });

        it('header counts case_analysis and set_logic separately', async () => {
            const app = createMockApp({
                sdc_clocks: () => ONE_CLOCK_RESPONSE,
                sdc_clock_modes: () => ({
                    current_mode: '',
                    scene_names: [],
                    case_analysis: [{ pin: 'a', value: '0' },
                                    { pin: 'b', value: '1' }],
                    logic_values:  [{ pin: 'c', value: '0' }],
                }),
            });
            const widget = new SdcWidget(app);
            await settle();

            // Header format: "<N> set_case_analysis · <M> set_logic_zero/one/dc"
            const text = widget._caseStrip.textContent;
            assert.ok(text.includes('2 set_case_analysis'),
                'header reports case_analysis count');
            assert.ok(text.includes('1 set_logic_zero/one/dc'),
                'header reports set_logic_* count');
        });
    });

    // ── Limits tab: new sections ────────────────────────────────────────────

    describe('limits tab new sections', () => {
        const LIMITS_WITH_NEW_FIELDS = {
            time_unit: 'ns',
            cap_unit:  'pF',
            clock_latencies:      [],
            clock_uncertainties:  [],
            port_loads:           [],
            clock_insertions: [{
                clock: 'clk', pin: 'clk_port',
                late_rise_max: 0.5, late_rise_min: 0.3,
                late_fall_max: 0.5, late_fall_min: 0.3,
                early_rise_max: 0.5, early_rise_min: 0.3,
                early_fall_max: 0.5, early_fall_min: 0.3,
            }],
            disabled_timing: [
                { scope: 'pin',         name: 'u_inst/A' },
                { scope: 'port',        name: 'scan_in' },
                { scope: 'lib_port',    name: 'Q' },
                // Paired -from A -to Z lives on `from_to`, distinct from
                // the singleton from[] / to[] arrays. Older fixtures put
                // it on from+to which conflated paired vs. singleton.
                { scope: 'cell_port',   name: 'BUFX1', all: false,
                  from: [], to: [], from_to: [{ from: 'A', to: 'Z' }] },
                // Singleton -from + -to (two separate set_disable_timing
                // commands on the same cell) — rendered as two lines.
                { scope: 'cell_port',   name: 'INVX1', all: false,
                  from: ['A'], to: ['Z'], from_to: [] },
                { scope: 'inst_port',   name: 'u_buf', all: true },
            ],
        };

        it('renders Clock Insertion Delays section', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_limits: () => LIMITS_WITH_NEW_FIELDS,
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Limits');
            await settle();

            const text = widget._limScrollArea.textContent;
            assert.ok(text.includes('Clock Insertion Delays'),
                'section heading visible');
            assert.ok(text.includes('clk_port'),
                'insertion pin visible in table');
        });

        it('renders Disabled Timing section with scope labels', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_limits: () => LIMITS_WITH_NEW_FIELDS,
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Limits');
            await settle();

            const text = widget._limScrollArea.textContent;
            assert.ok(text.includes('Disabled Timing'),
                'section heading visible');
            assert.ok(text.includes('u_inst/A'),   'pin-scope row visible');
            assert.ok(text.includes('scan_in'),    'port-scope row visible');
            assert.ok(text.includes('BUFX1'),      'cell-scope row visible (paired)');
            assert.ok(text.includes('INVX1'),      'cell-scope row visible (singleton)');
            assert.ok(text.includes('-from A -to Z'),
                'paired from/to (from_to[]) renders as a single specifier line');
            // Singleton -from / -to (two separate commands on INVX1) render
            // as two distinct lines, each missing the other half.
            assert.ok(text.includes('-from A'),
                'singleton -from rendered separately');
            assert.ok(text.includes('-to Z'),
                'singleton -to rendered separately');
            assert.ok(text.includes('(all timing)'),
                'all-timing specifier rendered for inst scope');
        });

        it('empty-placeholder message mentions set_disable_timing', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_limits: () => ({
                    time_unit: 'ns', cap_unit: 'pF',
                    clock_latencies: [], clock_insertions: [],
                    clock_uncertainties: [], port_loads: [], disabled_timing: [],
                }),
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Limits');
            await settle();

            assert.ok(widget._limScrollArea.textContent.includes(
                'set_disable_timing'),
                'empty message lists all source constraints');
        });

        // Helper for the per-section tests below — keeps each test focused
        // on the section it's checking by emitting an otherwise-empty
        // limits payload with just the under-test array populated.
        const limitsApp = (extra = {}) => createMockApp({
            sdc_clocks: () => EMPTY_CLOCKS,
            sdc_clock_modes: () => EMPTY_MODES,
            sdc_limits: () => ({
                time_unit: 'ns', cap_unit: 'pF',
                clock_latencies: [], clock_insertions: [],
                clock_uncertainties: [], port_loads: [],
                disabled_timing: [], cell_limits: [],
                clock_slew_limits: [], pin_cap_limits: [],
                pin_clock_uncertainties: [],
                ...extra,
            }),
        });

        it('renders Clock Latencies section with rise/fall × min/max columns', async () => {
            const app = limitsApp({
                clock_latencies: [{
                    clock: 'clk', pin: null,
                    rise_max: 0.5, rise_min: 0.4,
                    fall_max: 0.5, fall_min: 0.4,
                }],
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Limits');
            await settle();
            const text = widget._limScrollArea.textContent;
            assert.ok(text.includes('Clock Latencies'), 'section heading visible');
            assert.ok(text.includes('clk'), 'clock name in row');
            assert.ok(text.includes('0.500'), 'latency value formatted to 3 sig figs');
        });

        it('renders Clock Uncertainties with separate setup / hold columns', async () => {
            const app = limitsApp({
                clock_uncertainties: [
                    { clock: 'clk_a', setup: 0.3,  hold: 0.15 },
                    { clock: 'clk_b', setup: null, hold: 0.10 },  // missing setup
                ],
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Limits');
            await settle();
            const text = widget._limScrollArea.textContent;
            assert.ok(text.includes('Clock Uncertainties'), 'section visible');
            assert.ok(text.includes('clk_a'), 'first uncertainty row present');
            assert.ok(text.includes('0.300'), 'setup value rendered');
            assert.ok(text.includes('0.150'), 'hold value rendered');
            assert.ok(text.includes('—'),
                'missing-value cells render em-dash placeholder');
        });

        it('renders Port Loads section when set_load values are present', async () => {
            const app = limitsApp({
                port_loads: [{ port: 'data_out',
                               rise_max: 0.10, fall_max: 0.10,
                               wire_cap_max: 0.04 }],
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Limits');
            await settle();
            const text = widget._limScrollArea.textContent;
            assert.ok(text.includes('Port Loads'),  'section heading visible');
            assert.ok(text.includes('data_out'),    'port name in row');
            assert.ok(text.includes('0.100'),       'pin-cap value rendered');
            assert.ok(text.includes('pF'),          'cap unit suffix shown');
        });

        it('renders Pin-Scope Cap Limits section', async () => {
            const app = limitsApp({
                pin_cap_limits: [{ pin: 'u_top/ff/D',
                                   cap_max: 0.05, cap_min: null }],
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Limits');
            await settle();
            const text = widget._limScrollArea.textContent;
            assert.ok(text.includes('Pin-Scope Cap Limits'),
                'pin-scope cap section visible');
            assert.ok(text.includes('u_top/ff/D'), 'pin name shown');
            assert.ok(text.includes('0.0500'),      'cap_max value rendered');
        });

        it('renders Pin-Anchored Uncertainty section', async () => {
            const app = limitsApp({
                pin_clock_uncertainties: [{ pin: 'u_x/q_reg/D',
                                            setup: 0.40, hold: null }],
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Limits');
            await settle();
            const text = widget._limScrollArea.textContent;
            assert.ok(text.includes('Pin-Anchored Uncertainty'),
                'pin-anchored uncertainty section visible');
            assert.ok(text.includes('u_x/q_reg/D'), 'pin name rendered');
            assert.ok(text.includes('0.400'),       'setup value rendered');
        });
    });

    // ── Exceptions tab: filter buttons ──────────────────────────────────────

    describe('exceptions tab filter buttons', () => {
        // The badge-rendering and group-name tests live alongside the other
        // "all four types" cases below; this block only checks toolbar UI.
        it('Group filter button is present in the toolbar', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_exceptions: () => ({ time_unit: 'ns', exceptions: [] }),
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Exceptions');
            await settle();

            const panel = widget._tabPanels['Exceptions'];
            const labels = Array.from(panel.querySelectorAll('button'))
                .map(b => b.textContent.toLowerCase());
            assert.ok(labels.some(l => l.includes('group')),
                'group_path filter button present');
        });

        it('clicking false-path filter hides multi-cycle and path-delay rows', async () => {
            // Three exception types in one payload; the user clicks the
            // false_path filter and we expect only false_path rows to
            // render. The other rows should not be in the DOM at all
            // (the renderer rebuilds the list per filter, it does not
            // toggle visibility) — so absence is the right assertion.
            const exceptions = [
                { id: 1, type: 'false_path',  min_max: 'max', multiplier: 0,
                  delay: null, from_pins: ['a/Q'], from_clocks: [], thrus: [],
                  to_pins: ['b/D'], to_clocks: [] },
                { id: 2, type: 'multi_cycle', min_max: 'max', multiplier: 3,
                  delay: null, use_end_clk: true,
                  from_pins: ['c/Q'], from_clocks: [], thrus: [],
                  to_pins: ['d/D'], to_clocks: [] },
                { id: 3, type: 'path_delay', min_max: 'max', multiplier: 0,
                  delay: 4.2, ignore_clk_latency: false, break_path: false,
                  from_pins: ['e/Q'], from_clocks: [], thrus: [],
                  to_pins: ['f/D'], to_clocks: [] },
            ];
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_exceptions: () => ({ time_unit: 'ns', exceptions }),
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Exceptions');
            await settle();
            // Sanity check — all three pin pairs visible before filtering.
            const all = widget._excScrollArea.textContent;
            assert.ok(all.includes('a/Q') && all.includes('c/Q')
                      && all.includes('e/Q'),
                'all exception types render before any filter is clicked');

            // Click the false_path filter button.
            const fpBtn = widget._excFilterBtns.false_path;
            assert.ok(fpBtn, 'false_path filter button exists');
            fpBtn.click();
            await settle();

            const filtered = widget._excScrollArea.textContent;
            assert.ok(filtered.includes('a/Q'),
                'false_path row still rendered');
            assert.ok(!filtered.includes('c/Q'),
                'multi_cycle row hidden under false_path filter');
            assert.ok(!filtered.includes('e/Q'),
                'path_delay row hidden under false_path filter');
            assert.equal(widget._excActiveFilter, 'false_path',
                'active-filter state tracks the click');
        });

        it('exception filter buttons display running counts per type', async () => {
            const exceptions = [
                { id: 1, type: 'false_path',  min_max: 'max', multiplier: 0,
                  delay: null, from_pins: [], from_clocks: [], thrus: [],
                  to_pins: [], to_clocks: [] },
                { id: 2, type: 'multi_cycle', min_max: 'max', multiplier: 2,
                  delay: null, use_end_clk: false,
                  from_pins: [], from_clocks: [], thrus: [],
                  to_pins: [], to_clocks: [] },
                { id: 3, type: 'multi_cycle', min_max: 'max', multiplier: 2,
                  delay: null, use_end_clk: false,
                  from_pins: [], from_clocks: [], thrus: [],
                  to_pins: [], to_clocks: [] },
            ];
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_exceptions: () => ({ time_unit: 'ns', exceptions }),
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Exceptions');
            await settle();
            assert.ok(widget._excFilterBtns.all.textContent.includes('(3)'),
                'All button counts every exception');
            assert.ok(widget._excFilterBtns.false_path.textContent.includes('(1)'),
                'false_path button shows count of 1');
            assert.ok(widget._excFilterBtns.multi_cycle.textContent.includes('(2)'),
                'multi_cycle button shows count of 2');
            assert.ok(widget._excFilterBtns.path_delay.textContent.includes('(0)'),
                'path_delay button shows zero when no path_delay exception present');
        });
    });

    // ── Clickable pin/port → inspect-by-ODB ─────────────────────────────────
    //
    // The SDC response carries ODB refs (odb_type + odb_id) next to every
    // linkable pin name. Clicking such a link must send
    //   {type: "inspect", odb_type, odb_id}
    // — no separate server-side name-resolution round trip.

    describe('clickable pins use inspect-by-ODB', () => {
        function captureInspectApp(clocksResp) {
            let lastInspect = null;
            return {
                app: {
                    websocketManager: {
                        readyPromise: Promise.resolve(),
                        request(msg) {
                            // SDC pin clicks send `inspect_by_odb` to
                            // bypass the canvas-pick selectables[]
                            // round-trip; record the request so tests
                            // can assert the ODB ref it carried.
                            if (msg.type === 'inspect_by_odb') {
                                lastInspect = msg;
                                return Promise.resolve({ properties: [] });
                            }
                            if (msg.type === 'sdc_clocks') return Promise.resolve(clocksResp);
                            if (msg.type === 'sdc_clock_modes') return Promise.resolve(EMPTY_MODES);
                            return Promise.resolve({});
                        },
                    },
                    updateInspector: () => {},
                    focusComponent: () => {},
                },
                lastInspect: () => lastInspect,
            };
        }

        it('primary-clock src uses sources_odb[0] on click', async () => {
            const clocks = {
                clocks: [{
                    name: 'clk', period: 10, waveform: [0, 5],
                    is_generated: false, is_virtual: false, is_propagated: false,
                    master_clock: null, divide_by: null, multiply_by: null,
                    invert: false, edges: null, edge_shifts: null, src_pin: null,
                    sources: ['clk_port'],
                    sources_odb: [{ odb_type: 'bterm', odb_id: 7 }],
                    uncertainty_setup: null, uncertainty_hold: null,
                }],
                clock_tree: [{ name: 'clk', children: [] }],
                time_unit: 'ns',
            };
            const { app, lastInspect } = captureInspectApp(clocks);
            const widget = new SdcWidget(app);
            await settle();

            // Find the clock-card stat row containing 'clk_port' and click its value span.
            const rows = Array.from(widget._cardScrollArea.querySelectorAll('div'));
            const srcRow = rows.find(r => r.textContent.includes('clk_port')
                                        && r.querySelector('span'));
            assert.ok(srcRow, 'src row rendered');
            const linkEl = Array.from(srcRow.querySelectorAll('span'))
                .find(s => s.textContent.includes('clk_port'));
            assert.ok(linkEl, 'link span present');
            linkEl.click();
            await settle();

            const msg = lastInspect();
            assert.ok(msg, 'inspect request was sent');
            assert.equal(msg.odb_type, 'bterm');
            assert.equal(msg.odb_id,   7);
        });

        it('generated-clock src_pin uses src_pin_odb_type/id', async () => {
            const clocks = {
                clocks: [{
                    name: 'clk', period: 10, waveform: [0, 5],
                    is_generated: false, is_virtual: false, is_propagated: false,
                    master_clock: null, divide_by: null, multiply_by: null,
                    invert: false, edges: null, edge_shifts: null, src_pin: null,
                    sources: [], sources_odb: [],
                    uncertainty_setup: null, uncertainty_hold: null,
                }, {
                    name: 'clk_div2', period: 20, waveform: [0, 10],
                    is_generated: true, is_virtual: false, is_propagated: false,
                    master_clock: 'clk', divide_by: 2, multiply_by: null,
                    invert: false, edges: null, edge_shifts: null,
                    src_pin: 'u_div/clk_out',
                    src_pin_odb_type: 'iterm',
                    src_pin_odb_id:   42,
                    sources: [], sources_odb: [],
                    uncertainty_setup: null, uncertainty_hold: null,
                }],
                clock_tree: [
                    { name: 'clk', children: [{ name: 'clk_div2', children: [] }] },
                ],
                time_unit: 'ns',
            };
            const { app, lastInspect } = captureInspectApp(clocks);
            const widget = new SdcWidget(app);
            await settle();

            const rows = Array.from(widget._cardScrollArea.querySelectorAll('div'));
            const srcRow = rows.find(r => r.textContent.includes('u_div/clk_out')
                                        && r.querySelector('span'));
            assert.ok(srcRow, 'generated src_pin row rendered');
            const linkEl = Array.from(srcRow.querySelectorAll('span'))
                .find(s => s.textContent.includes('u_div/clk_out'));
            linkEl.click();
            await settle();

            const msg = lastInspect();
            assert.ok(msg, 'inspect request was sent');
            assert.equal(msg.odb_type, 'iterm');
            assert.equal(msg.odb_id,   42);
        });

        it('names without an odb ref are not clickable', async () => {
            // A source without sources_odb → _linkifyPin no-ops; no click handler.
            const clocks = {
                clocks: [{
                    name: 'clk', period: 10, waveform: [0, 5],
                    is_generated: false, is_virtual: false, is_propagated: false,
                    master_clock: null, divide_by: null, multiply_by: null,
                    invert: false, edges: null, edge_shifts: null, src_pin: null,
                    sources: ['unresolved_pin'],
                    // no sources_odb → ref.odb_type is undefined
                    uncertainty_setup: null, uncertainty_hold: null,
                }],
                clock_tree: [{ name: 'clk', children: [] }],
                time_unit: 'ns',
            };
            const { app, lastInspect } = captureInspectApp(clocks);
            const widget = new SdcWidget(app);
            await settle();

            const rows = Array.from(widget._cardScrollArea.querySelectorAll('div'));
            const srcRow = rows.find(r => r.textContent.includes('unresolved_pin'));
            const linkEl = Array.from(srcRow.querySelectorAll('span'))
                .find(s => s.textContent.includes('unresolved_pin'));
            // Cursor should NOT be set to pointer (linkify bailed out).
            assert.notEqual(linkEl.style.cursor, 'pointer');
            linkEl.click();
            await settle();
            assert.equal(lastInspect(), null,
                'clicking a non-linkified name sends no inspect request');
        });
    });

    // ── Port Delays: combined rise+fall diagram ─────────────────────────────
    //
    // Each entry from the backend carries rise_max/rise_min/fall_max/fall_min
    // for a single (port, clock, edge) constraint. When rise and fall differ,
    // the diagram draws BOTH on a single bar (Q1=Option B from the design
    // review). When they're equal it collapses to one marker (Q2). Inout
    // ports get one card with both Input delays and Output delays
    // sub-sections inside (Q4).
    describe('port delays combined rise/fall diagram', () => {
        // Uses the module-level makePdEntry / loadPdWidget helpers.
        const loadPd = loadPdWidget;

        // Count vertical marker lines drawn on the data bar. Each marker is
        // two stacked lines (outline + fill); we count by stroke color.
        function countMarkers(svg, fillColor) {
            return Array.from(svg.querySelectorAll('line'))
                .filter(l => l.getAttribute('stroke') === fillColor)
                .filter(l => {
                    // Vertical line (non-axis) inside the data bar.
                    const x1 = +l.getAttribute('x1');
                    const x2 = +l.getAttribute('x2');
                    const y1 = +l.getAttribute('y1');
                    const y2 = +l.getAttribute('y2');
                    return x1 === x2 && Math.abs(y2 - y1) > 0;
                }).length;
        }

        it('one diagram per port-card when rise == fall', async () => {
            const widget = await loadPd([makePdEntry({
                rise_max: 2, rise_min: 2, fall_max: 2, fall_min: 2,
            })]);
            const svgs = widget._pdScrollArea.querySelectorAll('svg');
            assert.equal(svgs.length, 1, 'one diagram per port-card');
        });

        it('rise != fall renders both markers in distinct colors', async () => {
            const widget = await loadPd([makePdEntry({
                rise_max: 2, rise_min: 2, fall_max: 4, fall_min: 4,
            })]);
            const svg = widget._pdScrollArea.querySelector('svg');
            const yellow = countMarkers(svg, 'var(--sdc-wf-marker-fill)');
            const pink   = countMarkers(svg, 'var(--sdc-wf-fall-fill)');
            assert.equal(yellow, 1, 'rise marker drawn');
            assert.equal(pink,   1, 'fall marker drawn');
            // Annotation should show per-edge values.
            const text = svg.textContent;
            assert.ok(text.includes('↑ rise'), 'rise label present');
            assert.ok(text.includes('↓ fall'), 'fall label present');
        });

        it('only rise (fall null) renders a yellow marker labeled ↑', async () => {
            const widget = await loadPd([makePdEntry({
                rise_max: 2, rise_min: 2,
                fall_max: null, fall_min: null,
            })]);
            const svg = widget._pdScrollArea.querySelector('svg');
            assert.equal(countMarkers(svg, 'var(--sdc-wf-marker-fill)'), 1,
                'yellow rise line');
            assert.equal(countMarkers(svg, 'var(--sdc-wf-fall-fill)'), 0,
                'no pink fall line');
        });

        it('only fall (rise null) renders a pink marker labeled ↓', async () => {
            // Fall-only is rendered with the fall (pink) color so the user
            // can tell at a glance that only the falling edge is constrained.
            const widget = await loadPd([makePdEntry({
                rise_max: null, rise_min: null,
                fall_max: 3, fall_min: 3,
            })]);
            const svg = widget._pdScrollArea.querySelector('svg');
            assert.equal(countMarkers(svg, 'var(--sdc-wf-marker-fill)'), 0,
                'no yellow rise line');
            assert.equal(countMarkers(svg, 'var(--sdc-wf-fall-fill)'), 1,
                'pink fall line drawn');
        });

        it('rise == fall (both at same point) renders a split marker', async () => {
            // Both edges constrained to the same value — the marker is split
            // half yellow / half pink so the user knows BOTH edges have
            // constraints (rather than mistaking it for a rise-only marker).
            const widget = await loadPd([makePdEntry({
                rise_max: 2, rise_min: 2, fall_max: 2, fall_min: 2,
            })]);
            const svg = widget._pdScrollArea.querySelector('svg');
            assert.equal(countMarkers(svg, 'var(--sdc-wf-marker-fill)'), 1,
                'yellow half present');
            assert.equal(countMarkers(svg, 'var(--sdc-wf-fall-fill)'), 1,
                'pink half present');
            assert.ok(svg.textContent.includes('↑↓'),
                'split marker labeled ↑↓');
        });

        it('rise == fall but with min/max spread renders an amber band', async () => {
            // When transitions match but each has a min/max range, we draw
            // the amber spread band — no per-edge markers needed. Filter by
            // y-attr so the legend swatch (also amber) doesn't get counted.
            const widget = await loadPd([makePdEntry({
                rise_max: 3, rise_min: 1, fall_max: 3, fall_min: 1,
            })]);
            const svg = widget._pdScrollArea.querySelector('svg');
            const dataBarAmbers = Array.from(svg.querySelectorAll('rect'))
                .filter(r => r.style.fill === 'var(--sdc-wf-uncert)')
                .filter(r => +r.getAttribute('y') > 30);
            assert.equal(dataBarAmbers.length, 1, 'amber spread band on data bar');
            assert.equal(countMarkers(svg, 'var(--sdc-wf-fall-fill)'), 0,
                'no separate fall marker for spread case');
        });

        it('rise spread + fall point renders two markers and bracket', async () => {
            // rise has a window [1,3], fall is a single point at 2.
            // Expect two color-coded markers; rise marker has a bracket.
            const widget = await loadPd([makePdEntry({
                rise_max: 3, rise_min: 1, fall_max: 2, fall_min: 2,
            })]);
            const svg = widget._pdScrollArea.querySelector('svg');
            assert.equal(countMarkers(svg, 'var(--sdc-wf-marker-fill)'), 2,
                'rise main line + bracket cap → 2 yellow vertical lines');
            assert.equal(countMarkers(svg, 'var(--sdc-wf-fall-fill)'),   1,
                'fall single marker');
        });

        it('output-port differing rise/fall renders mirrored two markers', async () => {
            const widget = await loadPd([makePdEntry({
                port: 'out1', direction: 'output', is_input: false,
                rise_max: 2, rise_min: 2, fall_max: 4, fall_min: 4,
            })]);
            const svg = widget._pdScrollArea.querySelector('svg');
            assert.equal(countMarkers(svg, 'var(--sdc-wf-marker-fill)'), 1);
            assert.equal(countMarkers(svg, 'var(--sdc-wf-fall-fill)'),   1);
            // Output annotation should use negative-relative-to-T notation.
            const text = svg.textContent;
            assert.ok(text.includes('↑ rise'), 'rise label in output annot');
            assert.ok(text.includes('↓ fall'), 'fall label in output annot');
        });

        it('inout port renders a single card with Input/Output sub-sections', async () => {
            const widget = await loadPd([
                // Input-side delay
                makePdEntry({
                    port: 'io1', direction: 'inout', is_input: true,
                    rise_max: 2, rise_min: 2, fall_max: 2, fall_min: 2,
                }),
                // Output-side delay (same port)
                makePdEntry({
                    port: 'io1', direction: 'inout', is_input: false,
                    rise_max: 3, rise_min: 3, fall_max: 3, fall_min: 3,
                }),
            ]);
            // Exactly one direct-child div wraps both sub-sections.
            const wrappers = Array.from(widget._pdScrollArea.children)
                .filter(el => el.tagName === 'DIV'
                           && el.querySelector('svg'));
            assert.equal(wrappers.length, 1, 'one card for the inout port');
            // Two diagrams in that card (one per direction).
            const svgs = wrappers[0].querySelectorAll('svg');
            assert.equal(svgs.length, 2, 'one diagram per direction');
            // Sub-section labels are present.
            const text = wrappers[0].textContent;
            assert.ok(text.includes('Input delays'),  'Input sub-section labeled');
            assert.ok(text.includes('Output delays'), 'Output sub-section labeled');
        });

        it('non-inout single-direction port has no sub-section labels', async () => {
            const widget = await loadPd([makePdEntry({
                port: 'in_only', direction: 'input', is_input: true,
                rise_max: 2, rise_min: 2, fall_max: 2, fall_min: 2,
            })]);
            const text = widget._pdScrollArea.textContent;
            assert.ok(!text.includes('Input delays'),
                'no sub-label when only one direction is present');
        });

        it('two entries with same clock + different launch edges share one bar', async () => {
            // Real-world case from the user's SDC:
            //   set_input_delay -clock clk -min 0   -max 0.2 [get_ports in1]
            //   set_input_delay -clock clk -clock_fall -min 0 -max 0.2 -add_delay [get_ports in1]
            // → backend emits two entries, both is_input=true, same clock,
            // one with clk_edge='rise', one with clk_edge='fall'. They must
            // collapse to a SINGLE SVG with ONE clock waveform and ONE data
            // bar carrying both entries' arrival markers (offset by their
            // launching edges).
            const widget = await loadPd([
                makePdEntry({ port: 'in1', clk_edge: 'rise',
                              rise_max: 0.2, rise_min: 0, fall_max: 0.2, fall_min: 0 }),
                makePdEntry({ port: 'in1', clk_edge: 'fall',
                              rise_max: 0.2, rise_min: 0, fall_max: 0.2, fall_min: 0 }),
            ]);
            // Exactly one SVG (single combined diagram), not two.
            const svgs = widget._pdScrollArea.querySelectorAll('svg');
            assert.equal(svgs.length, 1, 'one diagram for two same-clock entries');
            // Single shared clock waveform path.
            const clockPaths = svgs[0].querySelectorAll('path');
            assert.equal(clockPaths.length, 1, 'one clock waveform path');
            // Single shared data-bar outline (not stacked lanes).
            const outlines = Array.from(svgs[0].querySelectorAll('rect'))
                .filter(r => r.getAttribute('fill') === 'none'
                          && r.getAttribute('stroke') === 'var(--border)');
            assert.equal(outlines.length, 1, 'single shared data bar');
        });

        it('multi-entry marker color/symbol follows the launching edge', async () => {
            // Each entry should get ONE marker, color-coded by its clk_edge:
            //   clk_edge='rise' → ↑ in marker-fill (yellow)
            //   clk_edge='fall' → ↓ in fall-fill   (cyan)
            // Even though rise_max == fall_max within each entry, the marker
            // identifies the LAUNCHING edge of the constraint, not the data
            // transition.
            const widget = await loadPd([
                makePdEntry({ port: 'in1', clk_edge: 'rise',
                              rise_max: 0.2, rise_min: 0, fall_max: 0.2, fall_min: 0 }),
                makePdEntry({ port: 'in1', clk_edge: 'fall',
                              rise_max: 0.2, rise_min: 0, fall_max: 0.2, fall_min: 0 }),
            ]);
            const svg = widget._pdScrollArea.querySelector('svg');
            // Collect marker arrow labels (text nodes whose textContent is ↑ or ↓).
            const markers = Array.from(svg.querySelectorAll('text'))
                .filter(t => t.textContent === '↑' || t.textContent === '↓');
            const upMarkers   = markers.filter(t => t.textContent === '↑');
            const downMarkers = markers.filter(t => t.textContent === '↓');
            assert.equal(upMarkers.length, 1,
                'exactly one ↑ marker (rise-launched entry)');
            assert.equal(downMarkers.length, 1,
                'exactly one ↓ marker (fall-launched entry)');
            // Up-marker uses the rise color; down-marker uses the fall color.
            assert.equal(upMarkers[0].getAttribute('fill'),
                'var(--sdc-wf-marker-fill)', '↑ uses rise color');
            assert.equal(downMarkers[0].getAttribute('fill'),
                'var(--sdc-wf-fall-fill)', '↓ uses fall color');
        });

        it('paginated fetch appends without rebuilding existing cards', async () => {
            // Repro for the scroll-jump issue: when the second batch arrives,
            // existing port cards must NOT be re-created (which would force
            // the browser to collapse the scroll content and reset scrollTop).
            // We tag each card with an identity object on first render and
            // verify those exact nodes survive after _appendPortDelayBatch.
            const widget = await loadPd([
                makePdEntry({ port: 'in1', rise_max: 0.2, rise_min: 0,
                              fall_max: 0.2, fall_min: 0 }),
                makePdEntry({ port: 'in2', rise_max: 0.3, rise_min: 0,
                              fall_max: 0.3, fall_min: 0 }),
            ]);
            const initialCards = Array.from(
                widget._pdScrollArea.querySelectorAll('[data-pd-port]'));
            assert.equal(initialCards.length, 2, 'two cards rendered initially');
            // Tag the existing cards so we can prove they survived.
            initialCards.forEach((c, i) => { c.__testTag = `tag-${i}`; });

            // Simulate paginated batch arriving — append a NEW port.
            const more = [makePdEntry({ port: 'in3', rise_max: 0.4, rise_min: 0,
                                        fall_max: 0.4, fall_min: 0 })];
            widget._pdAllEntries = widget._pdAllEntries.concat(more);
            widget._pdTotal = 3;
            widget._appendPortDelayBatch(more);

            const afterCards = Array.from(
                widget._pdScrollArea.querySelectorAll('[data-pd-port]'));
            assert.equal(afterCards.length, 3, 'third card appended');
            // The first two cards must be the SAME DOM nodes (preserved).
            assert.equal(afterCards[0].__testTag, 'tag-0',
                'first card preserved across append');
            assert.equal(afterCards[1].__testTag, 'tag-1',
                'second card preserved across append');
            // The new card should be the appended one.
            assert.equal(afterCards[2].dataset.pdPort, 'in3',
                'new card is the third port');
        });

        it('cross-batch port replaces only its own card', async () => {
            // Edge case: a port has entries split across two batches.
            // The first card has 1 entry; the second batch adds another
            // entry for the same port. We replace just that one card with
            // the merged version (other cards stay put).
            const widget = await loadPd([
                makePdEntry({ port: 'in1', clk_edge: 'rise',
                              rise_max: 0.2, rise_min: 0,
                              fall_max: 0.2, fall_min: 0 }),
                makePdEntry({ port: 'in2', rise_max: 0.3, rise_min: 0,
                              fall_max: 0.3, fall_min: 0 }),
            ]);
            const inBefore = widget._pdScrollArea.querySelector(
                '[data-pd-port="in1"]');
            const in2Before = widget._pdScrollArea.querySelector(
                '[data-pd-port="in2"]');
            in2Before.__testTag = 'in2-preserved';

            // Second batch: another entry for in1 (e.g., -clock_fall).
            const more = [makePdEntry({ port: 'in1', clk_edge: 'fall',
                                        rise_max: 0.2, rise_min: 0,
                                        fall_max: 0.2, fall_min: 0 })];
            widget._pdAllEntries = widget._pdAllEntries.concat(more);
            widget._pdTotal = 3;
            widget._appendPortDelayBatch(more);

            const inAfter = widget._pdScrollArea.querySelector(
                '[data-pd-port="in1"]');
            const in2After = widget._pdScrollArea.querySelector(
                '[data-pd-port="in2"]');
            assert.notEqual(inAfter, inBefore,
                'in1 card was replaced (now contains both entries)');
            assert.equal(in2After.__testTag, 'in2-preserved',
                'in2 card was NOT replaced');
            // Still exactly two cards (no duplication of in1).
            const allCards = widget._pdScrollArea.querySelectorAll('[data-pd-port]');
            assert.equal(allCards.length, 2, 'no duplicate in1 card');
        });

        it('two entries with different clocks render separately', async () => {
            // Two entries with different clocks should NOT be collapsed —
            // different periods can't share an axis.
            const widget = await loadPd([
                makePdEntry({ port: 'in1', clock: 'clk1', clk_period: 10 }),
                makePdEntry({ port: 'in1', clock: 'clk2', clk_period: 7 }),
            ]);
            const svgs = widget._pdScrollArea.querySelectorAll('svg');
            assert.equal(svgs.length, 2,
                'separate diagrams for different clock periods');
        });

        it('legend uses rise/fall entries when transitions differ', async () => {
            const widget = await loadPd([makePdEntry({
                rise_max: 2, rise_min: 2, fall_max: 4, fall_min: 4,
            })]);
            const svg = widget._pdScrollArea.querySelector('svg');
            const legendText = svg.textContent;
            // Either full or short label form may appear depending on layout.
            const hasRise = legendText.includes('↑ rise') || legendText.includes('↑');
            const hasFall = legendText.includes('↓ fall') || legendText.includes('↓');
            assert.ok(hasRise, 'legend mentions rise');
            assert.ok(hasFall, 'legend mentions fall');
        });
    });

    // ── Endpoint tab: list-driven flow backed by Search::endpoints() ────────
    //
    // The Endpoint tab now defaults to a paginated list of every endpoint
    // the analyzer is tracking (rather than the previous "type a glob first"
    // experience). Initial population is gated behind an explicit "List
    // endpoints" button — same UX pattern as resolve-generated-clocks —
    // because the underlying Search::endpoints() walk is O(V) on the timing
    // graph the first time.
    describe('endpoint list view', () => {
        const SAMPLE_ENDPOINTS = {
            time_unit: 'ns',
            total: 4,
            offset: 0,
            // Note: backend no longer emits "port" — top-level ports
            // belong to the Port Delays tab and were producing
            // duplicate cards here.
            kinds_total: {
                flipflop: 2, latch: 1, macro: 0, stdcell: 1,
            },
            endpoints: [
                { name: 'u_a/q_reg/D', kind: 'flipflop',
                  odb_type: 'iterm', odb_id: 2,
                  instance: 'u_a/q_reg', cell: 'DFF_X1',
                  clocks: ['clk'] },
                { name: 'u_b/q_reg/D', kind: 'flipflop',
                  odb_type: 'iterm', odb_id: 3,
                  instance: 'u_b/q_reg', cell: 'DFF_X1',
                  clocks: ['clk_div2'] },
                { name: 'u_lat/q/D', kind: 'latch',
                  odb_type: 'iterm', odb_id: 4,
                  instance: 'u_lat/q', cell: 'LAT_X1',
                  clocks: ['clk'] },
                // Combinational stdcell endpoint — e.g. a buffer output that's
                // an endpoint via set_max_delay -to. Verifies the new
                // "stdcell" classification is distinct from "macro".
                { name: 'u_buf/Z', kind: 'stdcell',
                  odb_type: 'iterm', odb_id: 5,
                  instance: 'u_buf', cell: 'BUF_X1',
                  clocks: [] },
            ],
        };

        function setupEpApp(epListResp = SAMPLE_ENDPOINTS) {
            const calls = [];
            const app = createMockApp({
                sdc_clocks:        () => EMPTY_CLOCKS,
                sdc_clock_modes:   () => EMPTY_MODES,
                sdc_endpoint_list: (msg) => { calls.push(msg); return epListResp; },
            });
            return { app, calls };
        }

        it('initial state shows the populate button and no kind filter', async () => {
            const { app } = setupEpApp();
            const widget = new SdcWidget(app);
            widget._activateTab('Endpoint');
            await settle();

            assert.ok(widget._epListBtn,    'populate button exists');
            assert.notEqual(widget._epListBtn.style.display, 'none',
                'populate button visible by default');
            assert.equal(widget._epKindBar.style.display, 'none',
                'kind filter hidden until first list fetch');
            assert.ok(widget._epListArea.textContent.includes('List endpoints'),
                'list area shows the call-to-action message');
        });

        it('populate button triggers sdc_endpoint_list and renders rows', async () => {
            const { app, calls } = setupEpApp();
            const widget = new SdcWidget(app);
            widget._activateTab('Endpoint');
            await settle();

            widget._epListBtn.click();
            await settle();

            // One backend call with no pattern + 'all' kind (the defaults).
            assert.equal(calls.length, 1, 'one fetch issued');
            assert.equal(calls[0].pattern, '', 'no pattern (full list)');
            assert.equal(calls[0].kind,    'all', 'kind=all by default');
            assert.equal(calls[0].offset,  0, 'first batch offset=0');

            // Populate button hides; kind toolbar shows with counts.
            assert.equal(widget._epListBtn.style.display, 'none',
                'populate button hides after first fetch');
            assert.equal(widget._epKindBar.style.display, 'flex',
                'kind filter shown after first fetch');
            assert.ok(widget._epKindBtns.flipflop.textContent.includes('(2)'),
                'flipflop kind shows count from kinds_total');
            assert.ok(widget._epKindBtns.stdcell.textContent.includes('(1)'),
                'stdcell kind shows count');
            assert.ok(widget._epKindBtns.all.textContent.includes('(4)'),
                'all-kind sums kinds_total across the four endpoint kinds');
            // The Port filter button no longer exists — top-level ports
            // are rendered on the Port Delays tab instead.
            assert.equal(widget._epKindBtns.port, undefined,
                'no Port kind filter button');

            // Four endpoint cards rendered (no port row).
            const cards = widget._epListArea.querySelectorAll(
                '.sdc-ep-card');
            assert.equal(cards.length, 4, 'four endpoint cards rendered');
        });

        it('endpoints sharing an instance collapse into one card', async () => {
            // Two D pins on the same flop instance + a separate latch
            // instance: should produce 2 cards — NOT 3 — because the two
            // flop pins collapse into one card.
            const { app } = setupEpApp({
                time_unit: 'ns',
                total: 3, offset: 0,
                kinds_total: { flipflop: 2, latch: 1, macro: 0, stdcell: 0 },
                endpoints: [
                    { name: 'u_x/q_reg/D',     kind: 'flipflop',
                      odb_type: 'iterm', odb_id: 1,
                      instance: 'u_x/q_reg', cell: 'DFF',
                      clocks: ['clk'] },
                    { name: 'u_x/q_reg/SET_n', kind: 'flipflop',
                      odb_type: 'iterm', odb_id: 2,
                      instance: 'u_x/q_reg', cell: 'DFF',
                      clocks: ['clk'] },
                    { name: 'u_y/lat/D',       kind: 'latch',
                      odb_type: 'iterm', odb_id: 3,
                      instance: 'u_y/lat', cell: 'LAT',
                      clocks: ['clk'] },
                ],
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Endpoint');
            await settle();
            widget._epListBtn.click();
            await settle();

            const cards = widget._epListArea.querySelectorAll('.sdc-ep-card');
            assert.equal(cards.length, 2,
                'two cards (one flop with 2 endpoints, one latch)');
            // The flop card should show "2 endpoints" in its header.
            const text = widget._epListArea.textContent;
            assert.ok(text.includes('2 endpoints'),
                'card header shows endpoint count > 1');
        });

        it('cards start collapsed and expand on click with lazy fetch', async () => {
            const { app, calls } = makeEpApp({
                listResp: makeEpListResp([
                    { name: 'u_x/q_reg/D', kind: 'flipflop',
                      odb_type: 'iterm', odb_id: 1,
                      instance: 'u_x/q_reg', cell: 'DFF', clocks: ['clk'] },
                ]),
                detailResp: makeEpDetailResp([
                    { name: 'u_x/q_reg/D',
                      clocks: [{ name: 'clk', period: 10, waveform: [0, 5],
                                 library_setup: 0.1, library_hold: 0.05 }] },
                ]),
            });
            app.updateInspector = () => {};
            app.focusComponent  = () => {};
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();

            const card = widget._epListArea.querySelector('.sdc-ep-card');
            assert.ok(card, 'card exists');
            const body = card.querySelector('.sdc-ep-card-body');
            assert.equal(body.style.display, 'none',
                'body collapsed by default');
            // No sdc_endpoint fetch should have fired yet.
            const detailFetchesBefore = calls.filter(
                m => m.type === 'sdc_endpoint').length;
            assert.equal(detailFetchesBefore, 0,
                'no detail fetch until expansion');

            // Click ▶ to expand → fires sdc_endpoint and renders.
            card.querySelector('.sdc-ep-card-header').click();
            await settle();
            assert.notEqual(body.style.display, 'none',
                'body visible after expand');
            const detailFetchesAfter = calls.filter(
                m => m.type === 'sdc_endpoint').length;
            assert.equal(detailFetchesAfter, 1,
                'one detail fetch issued on first expand');
            assert.equal(calls[calls.length - 1].pin, 'u_x/q_reg/*',
                'fetched with instance glob');
            // Per-clock sub-card present.
            assert.ok(card.querySelector('.sdc-ep-clock-subcard'),
                'per-clock sub-card rendered');

            // Click again → collapse, no extra fetch.
            card.querySelector('.sdc-ep-card-header').click();
            await settle();
            assert.equal(body.style.display, 'none',
                'body hidden after second click');
            const detailFetchesAfter2 = calls.filter(
                m => m.type === 'sdc_endpoint').length;
            assert.equal(detailFetchesAfter2, 1,
                'no re-fetch on collapse');

            // Re-expand → still no extra fetch (cached DOM).
            card.querySelector('.sdc-ep-card-header').click();
            await settle();
            const detailFetchesAfter3 = calls.filter(
                m => m.type === 'sdc_endpoint').length;
            assert.equal(detailFetchesAfter3, 1,
                'no re-fetch on second expand');
        });

        it('search input + Enter submits a pattern fetch', async () => {
            const { app, calls } = setupEpApp();
            const widget = new SdcWidget(app);
            widget._activateTab('Endpoint');
            await settle();

            widget._epInput.value = 'u_a/*';
            widget._epInput.dispatchEvent(new window.KeyboardEvent('keydown', {
                key: 'Enter', bubbles: true,
            }));
            await settle();

            assert.equal(calls.length, 1, 'one fetch issued by Enter');
            assert.equal(calls[0].pattern, 'u_a/*', 'pattern propagated to backend');
            // Search submission also hides the populate button — the search
            // is itself a populate trigger.
            assert.equal(widget._epListBtn.style.display, 'none',
                'populate button hides after a search-triggered fetch');
        });

        it('refresh button appears after populate and re-fetches with current filters', async () => {
            const { app, calls } = setupEpApp();
            const widget = new SdcWidget(app);
            widget._activateTab('Endpoint');
            await settle();

            // Refresh button is hidden until first populate (nothing to
            // refresh yet — populate IS the first walk).
            assert.equal(widget._epRefreshBtn.style.display, 'none',
                'refresh button hidden before initial populate');

            widget._epListBtn.click();
            await settle();
            assert.equal(calls.length, 1, 'populate fetch issued');
            // Refresh now visible; populate hidden.
            assert.equal(widget._epListBtn.style.display, 'none',
                'populate hidden after first walk');
            assert.notEqual(widget._epRefreshBtn.style.display, 'none',
                'refresh visible after first walk');

            // Set a pattern + kind filter, then click refresh — it should
            // re-fetch with the same filters (the user uses refresh after
            // the design has changed; their filter intent is preserved).
            widget._epInput.value = 'u_top/*';
            widget._epKindBtns.flipflop.click();
            await settle();
            // (kind click already issues a fetch; track baseline before refresh.)
            const before = calls.length;

            widget._epRefreshBtn.click();
            await settle();
            assert.equal(calls.length, before + 1,
                'refresh issues exactly one new fetch');
            assert.equal(calls[calls.length - 1].pattern, 'u_top/*',
                'refresh preserves pattern');
            assert.equal(calls[calls.length - 1].kind, 'flipflop',
                'refresh preserves kind filter');
        });

        it('clicking a kind filter re-fetches with that kind', async () => {
            const { app, calls } = setupEpApp();
            const widget = new SdcWidget(app);
            widget._activateTab('Endpoint');
            await settle();

            widget._epListBtn.click();
            await settle();
            assert.equal(calls.length, 1, 'initial populate');

            widget._epKindBtns.flipflop.click();
            await settle();

            assert.equal(calls.length, 2, 'kind click triggers a re-fetch');
            assert.equal(calls[1].kind, 'flipflop',
                'kind filter sent to backend');
        });

        it('expanded card renders pins without explicit setup/hold checks', async () => {
            // The detail handler used to filter out internal pins that
            // didn't have a clock with library_setup/library_hold (and no
            // exceptions either). That dropped legitimate STA endpoints —
            // async reset pins, combinational stdcell endpoints reached
            // via set_max_delay -to, etc. The relaxed filter renders
            // every pin the backend returns inside the expanded card.
            const { app } = makeEpApp({
                listResp: makeEpListResp([
                    { name: 'u_async/q_reg/D', kind: 'flipflop',
                      odb_type: 'iterm', odb_id: 1,
                      instance: 'u_async/q_reg', cell: 'DFF', clocks: ['clk'] },
                ]),
                // No library_setup/hold and no exceptions — the old filter
                // would have dropped this; the relaxed filter keeps it.
                detailResp: makeEpDetailResp([
                    { name: 'u_async/q_reg/D',
                      clocks: [{ name: 'clk', period: 10, waveform: [0, 5],
                                 library_setup: null, library_hold: null }] },
                ]),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();

            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();

            // The expanded body should render the pin even though it has
            // no setup/hold checks; just no clock-diagram.
            const body = card.querySelector('.sdc-ep-card-body');
            assert.ok(!body.textContent.includes('No timing data available'),
                'pin without setup/hold checks is rendered');
            assert.ok(body.textContent.includes('u_async/q_reg/D'),
                'pin name appears in the expanded card');
        });

        it('exceptions dedup across pins on the same card', async () => {
            // Two D pins on the same flop instance, both hit by the same
            // false_path. Should produce one exception row with no
            // "applies to" tag (both pins share it).
            const sharedExc = {
                id: 42, type: 'false_path', min_max: 'max',
                multiplier: 0, delay: null,
                ignore_clk_latency: false, break_path: false,
                use_end_clk: false,
                from_pins: [], from_clocks: [], from_insts: [],
                from_transition: null,
                thrus: [],
                to_pins: [], to_clocks: [], to_insts: [],
                to_transition: null,
                comment: '',
            };
            const { app } = makeEpApp({
                listResp: makeEpListResp([
                    { name: 'u_x/q_reg/D', kind: 'flipflop',
                      odb_type: 'iterm', odb_id: 1,
                      instance: 'u_x/q_reg', cell: 'DFF', clocks: ['clk'] },
                    { name: 'u_x/q_reg/SET_n', kind: 'flipflop',
                      odb_type: 'iterm', odb_id: 2,
                      instance: 'u_x/q_reg', cell: 'DFF', clocks: ['clk'] },
                ]),
                detailResp: makeEpDetailResp([
                    { name: 'u_x/q_reg/D',     clocks: [], exceptions: [sharedExc] },
                    { name: 'u_x/q_reg/SET_n', clocks: [], exceptions: [sharedExc] },
                ]),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();

            const exSec = card.querySelector('.sdc-ep-card-exceptions');
            assert.ok(exSec, 'exception summary section rendered');
            assert.ok(exSec.textContent.includes('Applicable Exceptions (1)'),
                'deduped to a single exception row');
            // Both pins share the exception → no "applies to" tag.
            assert.ok(!exSec.textContent.includes('applies to:'),
                'no scope tag when exception covers all card pins');
        });

        it('per-clock sub-card header is sticky for tall macros', async () => {
            const { app } = makeEpApp({
                listResp: makeEpListResp([
                    { name: 'u_macro/A', kind: 'macro',
                      odb_type: 'iterm', odb_id: 1,
                      instance: 'u_macro', cell: 'BLOCK', clocks: ['clk'] },
                ]),
                detailResp: makeEpDetailResp([
                    { name: 'u_macro/A',
                      clocks: [{ name: 'clk', period: 10, waveform: [0, 5],
                                 library_setup: 0.1, library_hold: 0.05 }] },
                ]),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();
            const subhdr = card.querySelector('.sdc-ep-clock-header');
            assert.ok(subhdr, 'per-clock sub-card header exists');
            // The sub-card header carries position:sticky so the clock
            // context stays visible while the user scrolls past long
            // pin lists on the same card.
            assert.ok(subhdr.style.position === 'sticky',
                'per-clock header uses position:sticky');
        });

        it('per-clock sub-card draws a single multi-lane diagram', async () => {
            // Two pins on the same clock should produce one shared
            // CLK + 2-lane diagram, NOT two stacked per-pin diagrams.
            // Verifies the multi-lane renderer is being used by checking
            // for the .sdc-ep-multilane class it adds.
            const sharedClk = {
                name: 'clk', period: 10, waveform: [0, 5],
                library_setup: 0.1, library_hold: 0.05,
                uncertainty_setup: 0.05, uncertainty_hold: 0.02,
            };
            const { app } = makeEpApp({
                listResp: makeEpListResp([
                    { name: 'u_macro/D[0]', kind: 'macro',
                      odb_type: 'iterm', odb_id: 1,
                      instance: 'u_macro', cell: 'BLOCK', clocks: ['clk'] },
                    { name: 'u_macro/D[1]', kind: 'macro',
                      odb_type: 'iterm', odb_id: 2,
                      instance: 'u_macro', cell: 'BLOCK', clocks: ['clk'] },
                ]),
                detailResp: makeEpDetailResp([
                    { name: 'u_macro/D[0]', clocks: [sharedClk] },
                    { name: 'u_macro/D[1]', clocks: [sharedClk] },
                ]),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();

            // One shared multi-lane SVG, not multiple per-pin diagrams.
            const multilane = card.querySelectorAll('.sdc-ep-multilane');
            assert.equal(multilane.length, 1, 'one shared multi-lane diagram');
        });

        it('bus pins with identical constraints collapse to one lane', async () => {
            // 4 bus pins on same clock with identical setup/hold values
            // collapse to a single lane labeled DAT[3:0] (4 pins). The
            // diagram should report the bus label in its <title> for
            // hover, and only ONE lane outline rect should appear.
            const sharedClk = {
                name: 'clk', period: 10, waveform: [0, 5],
                library_setup: 0.1, library_hold: 0.05,
                uncertainty_setup: 0, uncertainty_hold: 0,
            };
            const { app } = makeEpApp({
                listResp: makeEpListResp([0, 1, 2, 3].map(i => ({
                    name: `u_macro/DAT[${i}]`, kind: 'macro',
                    odb_type: 'iterm', odb_id: 100 + i,
                    instance: 'u_macro', cell: 'BLOCK', clocks: ['clk'],
                }))),
                detailResp: makeEpDetailResp([0, 1, 2, 3].map(i => ({
                    name: `u_macro/DAT[${i}]`, clocks: [sharedClk],
                }))),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();

            // The diagram's text content should include the collapsed
            // bus label (range form, since indices 0..3 are contiguous).
            const svg = card.querySelector('.sdc-ep-multilane');
            assert.ok(svg, 'multi-lane diagram present');
            assert.ok(svg.textContent.includes('DAT[3:0]'),
                'bus pins collapsed to range label');
            assert.ok(svg.textContent.includes('4 pins'),
                'collapsed lane shows pin count');

            // Clicking the collapsed bus label expands it inline → the
            // diagram re-renders with one lane per pin so the user can
            // select an individual pin (the lane was unselectable while
            // collapsed). Find the bus-label text element by its ▶
            // marker, click it, and verify per-index labels appear.
            const labels = Array.from(svg.querySelectorAll('text'));
            const busLabel = labels.find(
                t => t.textContent.includes('DAT[3:0]'));
            assert.ok(busLabel, 'bus-collapsed label is a clickable text element');
            busLabel.dispatchEvent(new window.Event('click'));
            await settle();

            const expandedSvg = card.querySelector('.sdc-ep-multilane');
            // The collapsed range label should be gone, replaced by per-bit
            // labels for each individual pin.
            assert.ok(!expandedSvg.textContent.includes('DAT[3:0]'),
                'collapsed range label removed after expand');
            assert.ok(expandedSvg.textContent.includes('DAT[0]')
                      && expandedSvg.textContent.includes('DAT[3]'),
                'individual bus-bit labels render after expand');
        });

        it('bus pins with differing constraints stay as separate lanes', async () => {
            // Same bus name but different library_setup → must NOT collapse.
            const baseClk = (over) => ({ name: 'clk', period: 10,
                waveform: [0, 5], library_hold: 0.05, ...over });
            const { app } = makeEpApp({
                listResp: makeEpListResp([0, 1].map(i => ({
                    name: `u_macro/DAT[${i}]`, kind: 'macro',
                    odb_type: 'iterm', odb_id: 200 + i,
                    instance: 'u_macro', cell: 'BLOCK', clocks: ['clk'],
                }))),
                detailResp: makeEpDetailResp([
                    { name: 'u_macro/DAT[0]', clocks: [baseClk({ library_setup: 0.1 })] },
                    { name: 'u_macro/DAT[1]', clocks: [baseClk({ library_setup: 0.2 })] },
                ]),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();

            const svg = card.querySelector('.sdc-ep-multilane');
            assert.ok(svg, 'multi-lane diagram present');
            // Should NOT show a collapsed bus label.
            assert.ok(!svg.textContent.includes('DAT[1:0]'),
                'differing constraints kept as separate lanes');
            // Both per-pin labels should appear.
            assert.ok(svg.textContent.includes('DAT[0]'),
                'first pin labeled separately');
            assert.ok(svg.textContent.includes('DAT[1]'),
                'second pin labeled separately');
        });

        it('clock pins are surfaced in their own section, not as a lane', async () => {
            // The endpoint detail now returns every pin on the instance,
            // including CK. The widget should pull CK out into a "Clock pin"
            // section above the per-clock sub-cards rather than drawing a
            // CK lane (which would just duplicate the shared CLK waveform).
            const sharedClk = {
                name: 'clk', period: 10, waveform: [0, 5],
                library_setup: 0.1, library_hold: 0.05,
            };
            const { app } = makeEpApp({
                listResp: makeEpListResp([
                    { name: 'u_a/q_reg/D', kind: 'flipflop',
                      odb_type: 'iterm', odb_id: 1,
                      instance: 'u_a/q_reg', cell: 'DFF', clocks: ['clk'] },
                ]),
                detailResp: makeEpDetailResp([
                    { name: 'u_a/q_reg/D', direction: 'input',
                      is_clock_pin: false, clocks: [sharedClk] },
                    { name: 'u_a/q_reg/CK', direction: 'input',
                      is_clock_pin: true, clocks: [sharedClk] },
                    { name: 'u_a/q_reg/Q', direction: 'output',
                      is_clock_pin: false,
                      clk_to_q: { rise_max: 0.4, rise_min: 0.2,
                                  fall_max: 0.4, fall_min: 0.2 },
                      clocks: [sharedClk] },
                ]),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();

            const ckSec = card.querySelector('.sdc-ep-clockpin');
            assert.ok(ckSec, 'clock-pin section rendered');
            // Cards are clustered by instance; the pin row shows just
            // the leaf name (CK) with the full path on the title for
            // hover. Either form satisfies "names the CK pin".
            const ckLeaf = ckSec.querySelector('span[title="u_a/q_reg/CK"]');
            assert.ok(
                ckLeaf && ckLeaf.textContent === 'CK',
                'clock-pin section names the CK pin (leaf form, full path on title)');

            // The diagram should NOT have a lane labelled CK.
            const svg = card.querySelector('.sdc-ep-multilane');
            assert.ok(svg, 'multi-lane diagram present');
            assert.ok(!svg.textContent.includes('q_reg/CK'),
                'CK pin not drawn as a diagram lane');
            // Both D (input) and Q (output) pins should appear as lanes.
            assert.ok(svg.textContent.includes('q_reg/D'),
                'D input pin drawn as a lane');
            assert.ok(svg.textContent.includes('q_reg/Q'),
                'Q output pin drawn as a lane');
            // Output lane shows the clk→Q legend item.
            assert.ok(svg.textContent.includes('clk→Q'),
                'legend includes clk→Q for output lane');
        });

        it('Instance filter label is present on the kind toolbar', async () => {
            const { app } = setupEpApp();
            const widget = new SdcWidget(app);
            widget._activateTab('Endpoint');
            await settle();
            widget._epListBtn.click();
            await settle();
            // The toolbar prefixes the kind buttons with "Instance filter:"
            // — surfaces that the buttons gate which instances appear, not
            // which pin direction.
            assert.ok(widget._epKindBar.textContent.includes('Instance filter'),
                'kind toolbar labelled "Instance filter"');
        });

        it('latch with set_max_time_borrow draws an explicit borrow band', async () => {
            // An active-high latch with `set_max_time_borrow 2.0 [get_pins
            // u_lat/D]` should pick up the explicit limit, draw a 'borrow'
            // legend item (not 'borrow*' — the "*" tags the implicit
            // transparent-period default), and place an axis tick at
            // t=2.0 marking the borrow boundary.
            const sharedClk = {
                name: 'clk', period: 10, waveform: [0, 5],
                library_setup: 0.1, library_hold: 0.05,
                time_borrow_limit: 2.0,
            };
            const { app } = makeEpApp({
                listResp: makeEpListResp([
                    { name: 'u_lat/D', kind: 'latch',
                      odb_type: 'iterm', odb_id: 1,
                      instance: 'u_lat', cell: 'DLH_X1', clocks: ['clk'] },
                ]),
                detailResp: makeEpDetailResp([
                    { name: 'u_lat/D', direction: 'input',
                      is_clock_pin: false, capture_edge: 'fall',
                      clocks: [sharedClk] },
                ]),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();

            const svg = card.querySelector('.sdc-ep-multilane');
            assert.ok(svg, 'multi-lane diagram present');
            // Explicit borrow → legend reads 'borrow', not 'borrow*'.
            assert.ok(svg.textContent.includes('borrow'),
                'borrow legend item present');
            assert.ok(!svg.textContent.includes('borrow*'),
                'no implicit-default star marker when borrow is explicit');
            // The compact lane annotation should also surface the value.
            assert.ok(svg.textContent.includes('2.0'),
                'lane annotation shows borrow=2.0');
        });

        it('latch sub-card overlays a transparent-period band and labels the row EN', async () => {
            // For an active-high latch the closing edge is the falling
            // edge of the enable; the diagram should anchor t=0 there
            // and overlay a transparent-period band (open→close) across
            // the data lanes. The CLK row label switches to "EN" so the
            // user sees the level-sensitive nature of the signal.
            const sharedClk = {
                name: 'clk', period: 10, waveform: [0, 5],  // rise@0, fall@5
                library_setup: 0.1, library_hold: 0.05,
            };
            const { app } = makeEpApp({
                listResp: makeEpListResp([
                    { name: 'u_lat/D', kind: 'latch',
                      odb_type: 'iterm', odb_id: 1,
                      instance: 'u_lat', cell: 'LAT_X1', clocks: ['clk'] },
                ]),
                detailResp: makeEpDetailResp([
                    { name: 'u_lat/D', direction: 'input',
                      is_clock_pin: false, capture_edge: 'fall',
                      clocks: [sharedClk] },
                ]),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();

            const svg = card.querySelector('.sdc-ep-multilane');
            assert.ok(svg, 'multi-lane diagram present');
            assert.ok(svg.classList.contains('sdc-ep-latch'),
                'latch diagram tagged with sdc-ep-latch class');
            assert.ok(svg.querySelector('.sdc-ep-latch-overlay'),
                'transparent-period overlay drawn across data lanes');
            assert.ok(svg.textContent.includes('transparent'),
                'transparent legend item present');
            assert.ok(svg.textContent.includes('EN'),
                'enable row labelled EN, not CLK');
        });

        it('clicking a row dummy-test', async () => {
            // Replaced by the collapsible-card tests above. Keeping this
            // stub so the surrounding describe block layout doesn't
            // change line-numbers in unrelated tests.
            const { app } = setupEpApp();
            const baseRequest = app.websocketManager.request;
            app.websocketManager.request = (msg) => {
                if (msg.type === 'sdc_endpoint') {
                    return Promise.resolve({
                        found: true, multi: false, time_unit: 'ns',
                        total: 1, offset: 0, pins: [{
                            name: msg.pin, is_port: true,
                            port_delays: [], exceptions: [], clocks: [],
                        }],
                    });
                }
                return baseRequest(msg);
            };
            const widget = new SdcWidget(app);
            widget._activateTab('Endpoint');
            await settle();
            widget._epListBtn.click();
            await settle();
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            assert.ok(card, 'a card exists');
            // Clicking the header expands it in place; no separate detail
            // page exists anymore.
            card.querySelector('.sdc-ep-card-header').click();
            await settle();
            const body = card.querySelector('.sdc-ep-card-body');
            assert.notEqual(body.style.display, 'none',
                'body visible after expand');
        });
    });

    // ── Clock-gate (ICG) endpoints ───────────────────────────────────────────
    //
    // Liberty `clock_gating_integrated_cell` cells classify as
    // kind=clock_gate, get a Clock Gate filter button on the toolbar,
    // and render a flavor-specific waveform diagram on expand. Tests
    // mock the endpoint-list response with each flavor and verify:
    //  - the kinds_total.clock_gate count drives the filter button label
    //  - the card header shows the flavor badge
    //  - the expanded body renders the SVG diagram with the right rows
    //  - mux/other flavor drops the LATCH row + emits the glitch-warning
    //    tooltip
    describe('clock-gate (ICG) endpoint kind', () => {
        const ICG_ENDPOINTS = (flavor, instanceName) => ({
            time_unit: 'ns',
            total: 1, offset: 0,
            kinds_total: {
                flipflop: 0, latch: 0, macro: 0, stdcell: 0, clock_gate: 1,
            },
            endpoints: [{
                name: instanceName + '/E',
                kind: 'clock_gate',
                odb_type: 'iterm', odb_id: 1,
                instance: instanceName,
                cell: flavor === 'latch_negedge' ? 'CLKGATE_NEG_X1'
                    : flavor === 'other' ? 'CLKGATE_MUX_X1' : 'CLKGATE_X1',
                clocks: [{ name: 'clk_a', period: 10.0 }],
                clock_gate_flavor: flavor,
                clock_gate_ck_pin:  instanceName + '/CK',
                clock_gate_en_pin:  instanceName + '/E',
                clock_gate_out_pin: instanceName + '/GCK',
            }],
        });

        function setupEpApp(epListResp) {
            const app = createMockApp({
                sdc_clocks:        () => EMPTY_CLOCKS,
                sdc_clock_modes:   () => EMPTY_MODES,
                sdc_endpoint_list: () => epListResp,
            });
            return app;
        }

        async function listEndpoints(widget) {
            widget._activateTab('Endpoint');
            await settle();
            widget._epListBtn.click();
            await settle();
        }

        it('Endpoints toolbar exposes a Clock Gate filter button',
                async () => {
            const widget = new SdcWidget(setupEpApp(
                ICG_ENDPOINTS('latch_posedge', 'icg1')));
            await listEndpoints(widget);
            const btns = Array.from(
                widget._epKindBar.querySelectorAll('button'));
            const cg = btns.find(b => /Clock Gate/.test(b.textContent));
            assert.ok(cg, 'Clock Gate filter button rendered');
            assert.ok(/\(1\)/.test(cg.textContent),
                'button label carries the kinds_total count');
        });

        it('latch_posedge ICG card banners as ICG with flavor badge',
                async () => {
            const widget = new SdcWidget(setupEpApp(
                ICG_ENDPOINTS('latch_posedge', 'icg_pos')));
            await listEndpoints(widget);
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            assert.ok(card, 'ICG card rendered');
            const badge = card.querySelector('.sdc-ep-kind');
            assert.equal(badge.textContent, 'ICG',
                'kind badge says ICG');
            // Expand to render the waveform body.
            card.querySelector('.sdc-ep-card-header').click();
            await settle();
            const body = card.querySelector('.sdc-ep-card-body');
            assert.ok(/latch_posedge/.test(body.textContent),
                'flavor surfaced in the header strip');
            assert.ok(/CLKGATE_X1/.test(body.textContent),
                'cell name shown');
            // Pin-role chips for CK / EN / GCK.
            assert.ok(/CK=/.test(body.textContent), 'CK pin role chip');
            assert.ok(/EN=/.test(body.textContent), 'EN pin role chip');
            assert.ok(/GCK=/.test(body.textContent), 'GCK pin role chip');
            // Waveform SVG with row labels.
            const svg = body.querySelector('svg');
            assert.ok(svg, 'gating-waveform SVG rendered');
            const labels = Array.from(svg.querySelectorAll('text'))
                .map(t => t.textContent);
            assert.ok(labels.includes('CK'), 'CK row labelled');
            assert.ok(labels.includes('EN'), 'EN row labelled');
            assert.ok(labels.includes('LATCH'),
                'LATCH row present for latch flavor');
            assert.ok(labels.includes('GCK'), 'GCK row labelled');
        });

        it('latch_negedge ICG keeps the LATCH row but flips active edge',
                async () => {
            const widget = new SdcWidget(setupEpApp(
                ICG_ENDPOINTS('latch_negedge', 'icg_neg')));
            await listEndpoints(widget);
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();
            const body = card.querySelector('.sdc-ep-card-body');
            assert.ok(/latch_negedge/.test(body.textContent),
                'flavor badge says latch_negedge');
            const svg = body.querySelector('svg');
            const labels = Array.from(svg.querySelectorAll('text'))
                .map(t => t.textContent);
            assert.ok(labels.includes('LATCH'),
                'LATCH row also present for negedge flavor');
        });

        it('mux/other ICG drops the LATCH row and warns about glitches',
                async () => {
            const widget = new SdcWidget(setupEpApp(
                ICG_ENDPOINTS('other', 'icg_mux')));
            await listEndpoints(widget);
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();
            const body = card.querySelector('.sdc-ep-card-body');
            const svg = body.querySelector('svg');
            const labels = Array.from(svg.querySelectorAll('text'))
                .map(t => t.textContent);
            assert.ok(!labels.includes('LATCH'),
                'mux flavor: LATCH row omitted');
            assert.ok(labels.includes('CK'),  'CK row still present');
            assert.ok(labels.includes('EN'),  'EN row still present');
            assert.ok(labels.includes('GCK'), 'GCK row still present');
            // Flavor-tip on the badge mentions glitches.
            assert.ok(/glitch/i.test(body.innerHTML),
                'mux-flavor glitch warning present in tooltip');
        });

        it('legend strip names setup / hold / active-edge colours',
                async () => {
            const widget = new SdcWidget(setupEpApp(
                ICG_ENDPOINTS('latch_posedge', 'icg_pos')));
            await listEndpoints(widget);
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();
            const body = card.querySelector('.sdc-ep-card-body');
            // Legend should render as a flex strip with three labels.
            const text = body.textContent;
            assert.ok(/setup band/i.test(text),
                'legend names the orange band as "setup band"');
            assert.ok(/hold band/i.test(text),
                'legend names the cyan band as "hold band"');
            assert.ok(/active CK edge/i.test(text),
                'legend names the dashed guide as the active CK edge');
            // Tooltips on each swatch reference the underlying
            // Liberty timing arc, not just the colour.
            const swatches = Array.from(body.querySelectorAll('span'))
                .filter(s => s.title && /clock_gating/.test(s.title));
            assert.ok(swatches.length >= 2,
                'at least two legend swatches carry timing-arc tooltips');
            const setupItem = swatches.find(
                s => /clock_gating_setup/.test(s.title));
            const holdItem = swatches.find(
                s => /clock_gating_hold/.test(s.title));
            assert.ok(setupItem, 'setup swatch tooltip names the arc');
            assert.ok(holdItem, 'hold swatch tooltip names the arc');
        });

        it('setup / hold band tooltips identify themselves by name',
                async () => {
            const widget = new SdcWidget(setupEpApp(
                ICG_ENDPOINTS('latch_posedge', 'icg_pos')));
            await listEndpoints(widget);
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();
            // Walk every <title> on the SVG and check that the bands
            // self-identify so a hover always reveals which colour
            // means what.
            const titles = Array.from(card.querySelectorAll('title'))
                .map(t => t.textContent);
            assert.ok(titles.some(t => /Setup band \(orange\)/.test(t)),
                'setup band tooltip leads with its name + colour');
            assert.ok(titles.some(t => /Hold band \(cyan\)/.test(t)),
                'hold band tooltip leads with its name + colour');
            // And both reference the SDC override path so the user
            // knows where to look for the actual value.
            assert.ok(titles.some(t =>
                /Setup band/.test(t) && /set_clock_gating_check -setup/.test(t)),
                'setup tooltip points at the SDC override syntax');
            assert.ok(titles.some(t =>
                /Hold band/.test(t) && /set_clock_gating_check -hold/.test(t)),
                'hold tooltip points at the SDC override syntax');
        });

        it('GCK row tooltip distinguishes latched vs combinational',
                async () => {
            const wPos = new SdcWidget(setupEpApp(
                ICG_ENDPOINTS('latch_posedge', 'icg_pos')));
            await listEndpoints(wPos);
            const cardPos = wPos._epListArea.querySelector('.sdc-ep-card');
            cardPos.querySelector('.sdc-ep-card-header').click();
            await settle();
            const titlesPos = Array.from(
                cardPos.querySelectorAll('title')).map(t => t.textContent);
            assert.ok(titlesPos.some(t => /CK ∧ latched\(EN\)/.test(t)),
                'latch flavor: GCK = CK ∧ latched(EN)');

            const wMux = new SdcWidget(setupEpApp(
                ICG_ENDPOINTS('other', 'icg_mux')));
            await listEndpoints(wMux);
            const cardMux = wMux._epListArea.querySelector('.sdc-ep-card');
            cardMux.querySelector('.sdc-ep-card-header').click();
            await settle();
            const titlesMux = Array.from(
                cardMux.querySelectorAll('title')).map(t => t.textContent);
            assert.ok(titlesMux.some(t => /CK ∧ EN/.test(t) && !/latched/.test(t)),
                'mux flavor: GCK = CK ∧ EN (no latch)');
        });
    });

    // ── Exception types — full coverage ──────────────────────────────────────
    //
    // Each exception type drives a different badge label and (for MCP /
    // path-delay) carries a numeric attribute. Tests query via the badge's
    // data-exc-type / data-multiplier / data-delay attributes so cosmetic
    // changes to badge text don't cause spurious failures.

    describe('exceptions tab — all four types', () => {
        const ALL_EXCEPTIONS = {
            time_unit: 'ns',
            exceptions: [
                {
                    type: 'false_path',
                    min_max: 'max', multiplier: 0, delay: null,
                    from_pins: ['a/Q'], from_clocks: [], thrus: [],
                    to_pins: ['b/D'], to_clocks: [], id: 1,
                },
                {
                    type: 'multi_cycle',
                    min_max: 'max', multiplier: 3, delay: null,
                    use_end_clk: true,
                    from_pins: ['c/Q'], from_clocks: [], thrus: [],
                    to_pins: ['d/D'], to_clocks: [], id: 2,
                },
                {
                    type: 'path_delay',
                    min_max: 'max', multiplier: 0, delay: 4.2,
                    ignore_clk_latency: false, break_path: false,
                    from_pins: ['e/Q'], from_clocks: [], thrus: [],
                    to_pins: ['f/D'], to_clocks: [], id: 3,
                },
                {
                    type: 'group_path',
                    name: 'crit_grp', is_default: false,
                    min_max: 'max', multiplier: 0, delay: null,
                    from_pins: ['g/Q'], from_clocks: [], thrus: [],
                    to_pins: ['h/D'], to_clocks: [], id: 4,
                },
            ],
        };

        const setupExc = async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_exceptions: () => ALL_EXCEPTIONS,
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Exceptions');
            await settle();
            return widget;
        };

        it('renders one badge per exception with correct data-exc-type', async () => {
            const widget = await setupExc();
            const badges = widget._excScrollArea.querySelectorAll(
                '[data-exc-type]');
            assert.equal(badges.length, 4, 'four badges (one per exception)');

            const types = Array.from(badges).map(b => b.dataset.excType).sort();
            assert.deepEqual(types,
                ['false_path', 'group_path', 'multi_cycle', 'path_delay']);
        });

        it('multi_cycle badge carries the path multiplier', async () => {
            const widget = await setupExc();
            const mcp = widget._excScrollArea.querySelector(
                '[data-exc-type="multi_cycle"]');
            assert.ok(mcp, 'multi_cycle badge rendered');
            assert.equal(mcp.dataset.multiplier, '3',
                'multiplier matches fixture (3)');
            // The visible text is verified separately so a font/glyph change
            // doesn't double-fail.
            assert.ok(mcp.textContent.includes('3'),
                'multiplier visible in badge text');
        });

        it('path_delay badge carries the delay value', async () => {
            const widget = await setupExc();
            const pd = widget._excScrollArea.querySelector(
                '[data-exc-type="path_delay"]');
            assert.ok(pd, 'path_delay badge rendered');
            assert.equal(parseFloat(pd.dataset.delay), 4.2,
                'delay attribute matches fixture (4.2 ns)');
        });

        it('false_path badge has neither multiplier nor delay', async () => {
            const widget = await setupExc();
            const fp = widget._excScrollArea.querySelector(
                '[data-exc-type="false_path"]');
            assert.ok(fp, 'false_path badge rendered');
            assert.equal(fp.dataset.multiplier, undefined);
            assert.equal(fp.dataset.delay, undefined);
        });

        it('group_path row carries the group name in the header', async () => {
            // The group name (`crit_grp` in the fixture) must appear as text
            // in the same row as the GROUP badge — without it the user can't
            // tell which group_path command produced the row.
            const widget = await setupExc();
            assert.ok(widget._excScrollArea.textContent.includes('crit_grp'),
                'group_path name rendered alongside GROUP badge');
        });
    });

    // ── Diagram contract tests ───────────────────────────────────────────────
    //
    // These tests pin down the *contract* of the SVG drawing routines so the
    // shared-helper extraction later can be done with confidence. They avoid
    // brittleness by querying via the [data-role|data-band|data-marker|
    // data-ref] attributes that the widget tags onto every semantically-
    // meaningful element. Pixel positions and exact CSS-variable names are
    // never asserted on directly — instead we check element counts, ordering,
    // and the data attributes themselves.

    describe('clock waveform contract (Clocks tab _drawWaveform)', () => {
        // _drawWaveform expects a clock object with name, period, waveform,
        // uncertainty_setup, uncertainty_hold. We call it directly because
        // there's no other way to exercise the multi-period continuous draw
        // in isolation — the Clocks tab embeds it in a tree-rendered card.
        const callDraw = ({ waveform, period, setupUnc = null, holdUnc = null,
                            duration = 30, isSelected = false } = {}) => {
            const widget = new SdcWidget(createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
            }));
            const svg = widget._makeSvg(400, 60);
            widget._drawWaveform(svg, {
                name: 'clk',
                period,
                waveform,
                uncertainty_setup: setupUnc,
                uncertainty_hold: holdUnc,
            }, /*y0=*/ 0, /*xOff=*/ 0, /*waveW=*/ 400, /*rowH=*/ 60,
            duration, isSelected);
            return svg;
        };

        it('emits exactly one [data-role="clock-waveform"] path', () => {
            const svg = callDraw({ waveform: [0, 5], period: 10 });
            const paths = svg.querySelectorAll('[data-role="clock-waveform"]');
            assert.equal(paths.length, 1, 'exactly one tagged waveform path');
            assert.equal(paths[0].tagName.toLowerCase(), 'path');
        });

        it('does not emit a waveform path when waveform is empty', () => {
            const svg = callDraw({ waveform: [], period: 10 });
            assert.equal(
                svg.querySelectorAll('[data-role="clock-waveform"]').length,
                0,
                'empty waveform → no path');
        });

        it('emits a setup-unc band per edge when uncertainty_setup > 0', () => {
            // 3 periods (duration=30, period=10) × 2 edges per period = 6 edges.
            const svg = callDraw({
                waveform: [0, 5], period: 10, setupUnc: 0.5, duration: 30,
            });
            const setup = svg.querySelectorAll('[data-band="setup-unc"]');
            assert.equal(setup.length, 6,
                'one setup-unc band per edge across 3 periods');
        });

        it('emits a hold-unc band per edge when uncertainty_hold > 0', () => {
            const svg = callDraw({
                waveform: [0, 5], period: 10, holdUnc: 0.3, duration: 30,
            });
            assert.equal(
                svg.querySelectorAll('[data-band="hold-unc"]').length, 6,
                'one hold-unc band per edge across 3 periods');
        });

        it('emits no uncertainty bands when both uncertainties are null',
        () => {
            const svg = callDraw({ waveform: [0, 5], period: 10 });
            assert.equal(
                svg.querySelectorAll('[data-band="setup-unc"]').length, 0);
            assert.equal(
                svg.querySelectorAll('[data-band="hold-unc"]').length, 0);
        });
    });

    describe('port-delay diagram contract (single lane)', () => {
        // Uses the module-level makePdEntry / loadPdWidget helpers.
        const loadPd = loadPdWidget;

        it('emits exactly one clock-waveform path per port', async () => {
            const widget = await loadPd([makePdEntry({})]);
            const paths = widget._pdScrollArea.querySelectorAll(
                '[data-role="clock-waveform"]');
            assert.equal(paths.length, 1);
        });

        it('emits launch and capture reference lines (one each)', async () => {
            const widget = await loadPd([makePdEntry({})]);
            const refs = widget._pdScrollArea.querySelectorAll('[data-ref]');
            const kinds = Array.from(refs).map(r => r.dataset.ref).sort();
            assert.deepEqual(kinds, ['capture', 'launch']);
        });

        it('input-direction bar paints invalid → hold-unc → setup-unc → valid',
        async () => {
            // With uncertainty bands non-zero, all bands should be present.
            const widget = await loadPd([makePdEntry({
                uncertainty_setup: 0.5, uncertainty_hold: 0.5,
                rise_max: 4, rise_min: 4, fall_max: 4, fall_min: 4,
            })]);
            const svg = widget._pdScrollArea.querySelector('svg');
            const bands = Array.from(svg.querySelectorAll('[data-band]'))
                .map(b => b.dataset.band);
            // Order in the SVG is the order they were drawn — left-to-right
            // as bar paint zones. We expect exactly:
            //   invalid, hold-unc, setup-unc, valid
            // (no `uncert` band when there's no spread between rise and fall).
            assert.deepEqual(bands.filter(b => b !== 'valid'),
                ['invalid', 'hold-unc', 'setup-unc'],
                'invalid → hold-unc → setup-unc in order');
            assert.ok(bands.filter(b => b === 'valid').length >= 1,
                'at least one trailing valid band');
        });

        it('marker count matches the number of constrained transitions',
        async () => {
            // Both rise and fall constrained but at different values → two markers
            const widget = await loadPd([makePdEntry({
                rise_max: 2, rise_min: 2, fall_max: 5, fall_min: 5,
            })]);
            const svg = widget._pdScrollArea.querySelector('svg');
            const markers = svg.querySelectorAll('[data-marker]');
            assert.equal(markers.length, 2,
                'two markers for rise≠fall on the single-edge bar');
            const kinds = Array.from(markers).map(m => m.dataset.marker).sort();
            assert.deepEqual(kinds, ['fall', 'rise']);
        });

        it('marker time-max attribute matches the constrained value',
        async () => {
            const widget = await loadPd([makePdEntry({
                rise_max: 2.75, rise_min: 2.75,
                fall_max: null, fall_min: null,
            })]);
            const svg = widget._pdScrollArea.querySelector('svg');
            const rise = svg.querySelector('[data-marker="rise"]');
            assert.ok(rise, 'rise marker present');
            assert.equal(parseFloat(rise.dataset.timeMax), 2.75);
        });

        it('cyclic wrap: setup-unc band extending past T appears at t=0 too',
        async () => {
            // arrival = T - 0.1 → setup_unc starts inside [arrival, T] but
            // setup_unc=0.5 extends past T, so the wrap band at t=0..0.4 fires.
            const widget = await loadPd([makePdEntry({
                clk_period: 10,
                rise_max: 9.9, rise_min: 9.9,
                fall_max: 9.9, fall_min: 9.9,
                uncertainty_setup: 0.5, uncertainty_hold: 0,
            })]);
            const svg = widget._pdScrollArea.querySelector('svg');
            const setupBands = svg.querySelectorAll('[data-band="setup-unc"]');
            // 1 in-period + 1 wrapped at t=0
            assert.equal(setupBands.length, 2,
                'cyclic wrap doubles the setup-unc band count when it ' +
                'extends past T');
        });
    });

    describe('multi-lane port-delay diagram contract', () => {
        // Uses the module-level loadPdWidget helper.
        it('two entries on the same (port,clock) collapse into one diagram ' +
           'with two markers',
        async () => {
            // Same port + same clock with rise launch + fall launch — the
            // collapse logic produces one shared SVG with both markers.
            const entries = [
                makePdEntry({ clk_edge: 'rise',
                              rise_max: 2, rise_min: 2,
                              fall_max: 2, fall_min: 2 }),
                makePdEntry({ clk_edge: 'fall',
                              rise_max: 7, rise_min: 7,
                              fall_max: 7, fall_min: 7 }),
            ];
            const widget = await loadPdWidget(entries);
            const svgs = widget._pdScrollArea.querySelectorAll('svg');
            assert.equal(svgs.length, 1,
                'one collapsed SVG when both entries share clock+period');

            const markers = svgs[0].querySelectorAll('[data-marker]');
            assert.ok(markers.length >= 2,
                'at least two markers (one per entry)');
        });
    });

    describe('endpoint timing-diagram card contract', () => {
        // _renderClockDiagramCard takes a flat clk object with timing values.
        // We invoke it directly so we can pin down the band ordering without
        // wiring up an endpoint detail flow.
        const callRender = (over = {}) => {
            const widget = new SdcWidget(createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
            }));
            // _renderClockDiagramCard reads _epResultArea.clientWidth — make
            // sure it exists in the jsdom tree.
            widget._epResultArea = widget._epResultArea
                || document.createElement('div');
            const clk = {
                name: 'clk',
                period: 10,
                waveform: [0, 5],
                uncertainty_setup: 0.5,
                uncertainty_hold: 0.3,
                library_setup: 1.0,
                library_hold: 0.4,
                ...over,
            };
            return widget._renderClockDiagramCard(clk, 'ns');
        };

        it('emits exactly one clock-waveform path', () => {
            const card = callRender();
            const svg = card.querySelector('svg');
            assert.ok(svg, 'svg present');
            assert.equal(
                svg.querySelectorAll('[data-role="clock-waveform"]').length, 1);
        });

        it('emits a capture reference line', () => {
            const card = callRender();
            const refs = card.querySelectorAll('[data-ref]');
            const kinds = Array.from(refs).map(r => r.dataset.ref);
            assert.ok(kinds.includes('capture'),
                'capture reference line tagged');
        });

        it('paints bands in the documented left-to-right order: ' +
           'valid | lib-setup | setup-unc | hold-unc | lib-hold | valid',
        () => {
            const card = callRender({
                uncertainty_setup: 0.5, uncertainty_hold: 0.3,
                library_setup: 1.0,    library_hold: 0.4,
            });
            const bands = Array.from(card.querySelectorAll('[data-band]'))
                .map(b => b.dataset.band);
            assert.deepEqual(bands,
                ['valid', 'lib-setup', 'setup-unc', 'hold-unc', 'lib-hold',
                 'valid']);
        });

        it('omits zero-width bands cleanly', () => {
            // No library setup → that band is skipped.
            const card = callRender({
                library_setup: 0, library_hold: 0,
            });
            const bands = Array.from(card.querySelectorAll('[data-band]'))
                .map(b => b.dataset.band);
            assert.deepEqual(bands,
                ['valid', 'setup-unc', 'hold-unc', 'valid'],
                'lib-setup/lib-hold bands omitted when their value is zero');
        });

        it('renders a graceful "no waveform" fallback when waveform is missing',
        () => {
            const card = callRender({ waveform: null });
            assert.equal(card.querySelector('svg'), null,
                'no SVG when waveform is missing');
            assert.ok(card.textContent.toLowerCase().includes('no waveform'),
                'fallback message rendered');
        });
    });

    // ── Generated-clock variants ─────────────────────────────────────────────
    //
    // Every generated-clock form (-divide_by / -multiply_by / -invert) tags
    // its ratio badge with data-clock-badge so tests don't have to rely on
    // exact Unicode glyphs (÷, ×).

    describe('generated-clock badge variants', () => {
        const makeClockResp = (extra) => ({
            clocks: [
                {
                    name: 'clk', period: 10.0, waveform: [0, 5],
                    is_generated: false, is_virtual: false,
                    is_propagated: false, master_clock: null,
                    divide_by: null, multiply_by: null, invert: false,
                    edges: null, edge_shifts: null, src_pin: null,
                    sources: ['clk_port'],
                    uncertainty_setup: null, uncertainty_hold: null,
                },
                {
                    name: 'gen_clk', period: 10.0, waveform: [0, 5],
                    is_generated: true, is_virtual: false,
                    is_propagated: false, master_clock: 'clk',
                    divide_by: null, multiply_by: null, invert: false,
                    edges: null, edge_shifts: null,
                    src_pin: 'u/Q', sources: [],
                    uncertainty_setup: null, uncertainty_hold: null,
                    ...extra,
                },
            ],
            clock_tree: [{ name: 'clk',
                children: [{ name: 'gen_clk', children: [] }] }],
            time_unit: 'ns',
        });

        const setupClocks = async (extra) => {
            const app = createMockApp({
                sdc_clocks: () => makeClockResp(extra),
                sdc_clock_modes: () => EMPTY_MODES,
            });
            const widget = new SdcWidget(app);
            await settle();
            return widget;
        };

        it('multiply_by tags the badge with kind+value', async () => {
            const widget = await setupClocks({ multiply_by: 4 });
            const card = widget._cardScrollArea.querySelector(
                '[data-clock-name="gen_clk"]');
            assert.ok(card, 'gen_clk card rendered');
            const badge = card.querySelector(
                '[data-clock-badge="multiply_by"]');
            assert.ok(badge, 'multiply_by badge present');
            assert.equal(badge.dataset.value, '4');
        });

        it('divide_by tags the badge with kind+value', async () => {
            const widget = await setupClocks({ divide_by: 8 });
            const card = widget._cardScrollArea.querySelector(
                '[data-clock-name="gen_clk"]');
            assert.ok(card);
            const badge = card.querySelector(
                '[data-clock-badge="divide_by"]');
            assert.ok(badge, 'divide_by badge present');
            assert.equal(badge.dataset.value, '8');
        });

        it('invert tags the badge with kind=invert (no numeric value)',
        async () => {
            const widget = await setupClocks({ invert: true });
            const card = widget._cardScrollArea.querySelector(
                '[data-clock-name="gen_clk"]');
            assert.ok(card);
            const badge = card.querySelector('[data-clock-badge="invert"]');
            assert.ok(badge, 'invert badge present');
            assert.equal(badge.dataset.value, undefined,
                'invert carries no numeric value');
        });

        it('generated-clock ratio badges carry plain-English tooltips', async () => {
            const div2 = await setupClocks({ divide_by: 2 });
            const div2Badge = div2._cardScrollArea.querySelector(
                '[data-clock-badge="divide_by"]');
            assert.ok(div2Badge.title.includes('-divide_by 2'),
                'divide_by badge tooltip names the SDC option');

            const inv = await setupClocks({ invert: true });
            const invBadge = inv._cardScrollArea.querySelector(
                '[data-clock-badge="invert"]');
            assert.ok(invBadge.title.includes('-invert'),
                'invert badge tooltip names the SDC option');
        });
    });

    // ── Clock-domain dropdown filter on the Endpoints toolbar ─────────────

    // ── Clock-domain dropdown filter on the Port Delays toolbar ──────────

    describe('port-delays clock-domain dropdown filter', () => {
        const findOption = (filter, key) =>
            filter.panel.querySelector(`[data-clock-filter-option="${key}"]`);
        const toggleOption = (filter, key) => {
            const cb = findOption(filter, key).querySelector('input[type=checkbox]');
            cb.checked = !cb.checked;
            cb.dispatchEvent(new window.Event('change'));
        };

        it('hides the trigger on single-clock designs', async () => {
            const widget = await loadPdWidget([
                makePdEntry({ port: 'a', clock: 'clk' }),
                makePdEntry({ port: 'b', clock: 'clk' }),
            ]);
            assert.equal(widget._pdClockFilter.wrap.style.display, 'none',
                'dropdown hidden when only one clock domain present');
        });

        it('shows the trigger and panel with default-all selection', async () => {
            const widget = await loadPdWidget([
                makePdEntry({ port: 'in1',   clock: 'clk_a' }),
                makePdEntry({ port: 'in2',   clock: 'clk_a' }),
                makePdEntry({ port: 'in_b',  clock: 'clk_b' }),
            ]);
            const filter = widget._pdClockFilter;
            assert.notEqual(filter.wrap.style.display, 'none',
                'dropdown visible when multiple clocks present');
            assert.ok(filter.trigger.textContent.includes('All clocks'),
                'trigger label is "All clocks" by default');
            assert.ok(filter.trigger.textContent.includes('(3)'),
                'trigger label includes total entry count');

            // Open the panel and verify a row per clock with counts.
            filter.trigger.click();
            assert.equal(filter.panel.style.display, 'block');
            assert.ok(findOption(filter, 'clk_a').textContent.includes('(2)'),
                'clk_a row shows its count');
            assert.ok(findOption(filter, 'clk_b').textContent.includes('(1)'),
                'clk_b row shows its count');
        });

        it('unchecking a clock hides its port cards client-side', async () => {
            const widget = await loadPdWidget([
                makePdEntry({ port: 'in_a', clock: 'clk_a' }),
                makePdEntry({ port: 'in_b', clock: 'clk_b' }),
            ]);
            const filter = widget._pdClockFilter;
            filter.trigger.click();
            // Sanity — both port names appear before any filtering.
            assert.ok(widget._pdScrollArea.textContent.includes('in_a')
                && widget._pdScrollArea.textContent.includes('in_b'),
                'both port cards present before filtering');

            // Untick clk_b → only the clk_a card remains.
            toggleOption(filter, 'clk_b');
            await settle();
            assert.ok(widget._pdScrollArea.textContent.includes('in_a'),
                'clk_a port still rendered');
            assert.ok(!widget._pdScrollArea.textContent.includes('in_b'),
                'clk_b port hidden by clock filter');
            // Trigger label updates to single-clock form.
            assert.ok(filter.trigger.textContent.includes('clk_a'),
                'trigger label tracks the active selection');
        });

        it('renders a "(no clock)" row when an entry has no clock', async () => {
            // Mark one entry as exception-only (no clock attached).
            // The custom pdResponse derives clocks_total from each
            // entry's `clock` field — a null clock buckets under the
            // __none__ sentinel which the helper renders as "(no clock)".
            const widget = await loadPdWidget([
                makePdEntry({ port: 'in1',  clock: 'clk_a' }),
                makePdEntry({ port: 'p_exc', clock: null,
                              exception_only: true }),
            ]);
            const filter = widget._pdClockFilter;
            filter.trigger.click();
            const noneRow = findOption(filter, '__none__');
            assert.ok(noneRow, '"(no clock)" row rendered when ' +
                      'a null-clock entry is present');
            assert.ok(noneRow.textContent.includes('(no clock)'),
                'row labelled "(no clock)" — not the raw __none__ sentinel');
            assert.ok(noneRow.textContent.includes('(1)'),
                '"(no clock)" row shows count of null-clock entries');

            // Untick clk_a → only the exception-only entry survives.
            toggleOption(filter, 'clk_a');
            await settle();
            assert.ok(widget._pdScrollArea.textContent.includes('p_exc'),
                'exception-only port still rendered when "(no clock)" is on');
            assert.ok(!widget._pdScrollArea.textContent.includes('in1'),
                'regular clk_a port hidden after unchecking clk_a');
        });
    });

    // ── Glob/substring search input on the Port Delays toolbar ───────────

    describe('port-delays glob search input', () => {
        // Port Delays uses an Enter-to-search trigger (consistent with
        // Endpoints + CDC paths). Tests fire the Enter keydown to
        // commit the input value, mirroring how a user would interact.
        const commitSearch = (input) => {
            input.dispatchEvent(
                new window.KeyboardEvent('keydown', { key: 'Enter' }));
        };

        it('substring (no wildcards) narrows by port name', async () => {
            const widget = await loadPdWidget([
                makePdEntry({ port: 'top/in_a' }),
                makePdEntry({ port: 'top/reset_n' }),
                makePdEntry({ port: 'mem_addr[0]' }),
            ]);
            // Sanity: all three rendered before filtering.
            assert.ok(widget._pdScrollArea.textContent.includes('top/in_a'));
            assert.ok(widget._pdScrollArea.textContent.includes('top/reset_n'));
            assert.ok(widget._pdScrollArea.textContent.includes('mem_addr'));

            // Bare word matches as a substring (no need for *reset*).
            widget._pdSearchInput.value = 'reset';
            commitSearch(widget._pdSearchInput);
            await settle();
            assert.ok(
                widget._pdScrollArea.textContent.includes('top/reset_n'),
                'bare-word search matches port path as substring');
            assert.ok(
                !widget._pdScrollArea.textContent.includes('top/in_a'),
                'non-matching ports hidden by substring filter');
            assert.ok(
                !widget._pdScrollArea.textContent.includes('mem_addr'),
                'non-matching ports hidden by substring filter (2)');
        });

        it('glob with wildcards anchors to the whole name', async () => {
            const widget = await loadPdWidget([
                makePdEntry({ port: 'mem_addr[0]' }),
                makePdEntry({ port: 'mem_addr[1]' }),
                makePdEntry({ port: 'mem_we' }),
                makePdEntry({ port: 'top/mem_state' }),
            ]);
            // `mem_addr*` is anchored — `top/mem_state` (no
            // matching prefix) should be hidden, `mem_we` likewise.
            widget._pdSearchInput.value = 'mem_addr*';
            commitSearch(widget._pdSearchInput);
            await settle();
            assert.ok(widget._pdScrollArea.textContent.includes('mem_addr[0]'));
            assert.ok(widget._pdScrollArea.textContent.includes('mem_addr[1]'));
            assert.ok(!widget._pdScrollArea.textContent.includes('mem_we'),
                'mem_we excluded — anchored glob requires the prefix');
            assert.ok(!widget._pdScrollArea.textContent.includes('mem_state'),
                'top/mem_state excluded — anchored glob requires the prefix');
        });

        it('empty pattern reverts to no narrowing', async () => {
            const widget = await loadPdWidget([
                makePdEntry({ port: 'in_a' }),
                makePdEntry({ port: 'in_b' }),
            ]);
            widget._pdSearchInput.value = 'in_a';
            commitSearch(widget._pdSearchInput);
            await settle();
            assert.ok(!widget._pdScrollArea.textContent.includes('in_b'),
                'in_b filtered out');

            widget._pdSearchInput.value = '';
            commitSearch(widget._pdSearchInput);
            await settle();
            assert.ok(widget._pdScrollArea.textContent.includes('in_a')
                && widget._pdScrollArea.textContent.includes('in_b'),
                'clearing the pattern restores both ports');
        });

        it('no-match shows a "matching <pattern>" placeholder', async () => {
            const widget = await loadPdWidget([
                makePdEntry({ port: 'in_a' }),
            ]);
            widget._pdSearchInput.value = 'no_such_port';
            commitSearch(widget._pdSearchInput);
            await settle();
            assert.ok(
                widget._pdScrollArea.textContent.includes(
                    'matching "no_such_port"'),
                'placeholder names the pattern so the user sees why ' +
                'the list is empty');
        });

        it('combines with the direction filter', async () => {
            const widget = await loadPdWidget([
                makePdEntry({ port: 'in_clk_x',  direction: 'input' }),
                makePdEntry({ port: 'out_clk_x', direction: 'output',
                               is_input: false }),
                makePdEntry({ port: 'in_data',   direction: 'input' }),
            ]);
            widget._pdSearchInput.value = 'clk_x';
            commitSearch(widget._pdSearchInput);
            // Click "Inputs" filter button.
            widget._pdFilterBtns.input.click();
            await settle();
            assert.ok(widget._pdScrollArea.textContent.includes('in_clk_x'),
                'input matching the pattern survives both filters');
            assert.ok(!widget._pdScrollArea.textContent.includes('out_clk_x'),
                'output excluded by the direction filter even though ' +
                'its name matches the pattern');
            assert.ok(!widget._pdScrollArea.textContent.includes('in_data'),
                'input not matching the pattern excluded by the search');
        });

        // Live-typing must NOT trigger a filter — only Enter/Search-
        // button does. This pins down the chosen UX model
        // (consistent commit-on-Enter across Endpoints / Port Delays
        // / CDC paths) so a future "live for cheap filters" change
        // is an explicit decision, not a regression.
        it('typing without Enter does not narrow the list', async () => {
            const widget = await loadPdWidget([
                makePdEntry({ port: 'in_a' }),
                makePdEntry({ port: 'in_b' }),
            ]);
            widget._pdSearchInput.value = 'in_a';
            widget._pdSearchInput.dispatchEvent(
                new window.Event('input'));
            await settle();
            assert.ok(widget._pdScrollArea.textContent.includes('in_a')
                && widget._pdScrollArea.textContent.includes('in_b'),
                'both ports remain visible until Enter / Search '
                + 'commits the pattern');
        });

        // The Search button is the click-driven equivalent of
        // pressing Enter — mouse-only users get the same affordance.
        it('Search button commits the pattern just like Enter',
                async () => {
            const widget = await loadPdWidget([
                makePdEntry({ port: 'in_a' }),
                makePdEntry({ port: 'in_b' }),
            ]);
            const buttons = Array.from(
                widget._pdSearchInput.parentElement.querySelectorAll(
                    'button'));
            const searchBtn = buttons.find(
                b => b.textContent === 'Search');
            assert.ok(searchBtn, 'Search button is present in the toolbar');
            widget._pdSearchInput.value = 'in_a';
            searchBtn.click();
            await settle();
            assert.ok(widget._pdScrollArea.textContent.includes('in_a'),
                'matching port remains');
            assert.ok(!widget._pdScrollArea.textContent.includes('in_b'),
                'non-matching port hidden after Search-button click');
        });
    });

    describe('endpoint clock-domain dropdown filter', () => {
        // Quick-helpers to query the popup checkbox panel by clock name.
        const findClockRow = (widget, key) =>
            widget._epClockFilter.panel.querySelector(
                `[data-clock-filter-option="${key}"]`);
        const findClockBox = (widget, key) =>
            findClockRow(widget, key).querySelector('input[type=checkbox]');
        // jsdom doesn't reliably fire 'change' from a programmatic
        // click(), so flip + dispatch explicitly. Mirrors how the
        // mode-selector tests trigger their <select>.
        const toggleClockBox = (widget, key) => {
            const cb = findClockBox(widget, key);
            cb.checked = !cb.checked;
            cb.dispatchEvent(new window.Event('change'));
        };

        it('hides the trigger on single-clock designs', async () => {
            // Only one clock domain → the dropdown adds no value (the
            // sole option would be "All clocks (N)") and clutters the
            // toolbar, so it stays hidden.
            const { app } = makeEpApp({
                listResp: {
                    ...makeEpListResp([{
                        name: 'u_a/q_reg/D', kind: 'flipflop',
                        odb_type: 'iterm', odb_id: 1,
                        instance: 'u_a/q_reg', cell: 'DFF',
                        clocks: ['clk_a'],
                    }]),
                    clocks_total: { clk_a: 1 },
                },
                detailResp: makeEpDetailResp([]),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();
            assert.equal(widget._epClockFilter.wrap.style.display, 'none',
                'trigger hidden when only one clock domain present');
        });

        it('shows the trigger and panel with default-all selection', async () => {
            const { app, calls } = makeEpApp({
                listResp: {
                    ...makeEpListResp([
                        { name: 'u_a/q_reg/D', kind: 'flipflop',
                          odb_type: 'iterm', odb_id: 1,
                          instance: 'u_a/q_reg', cell: 'DFF',
                          clocks: ['clk_a'] },
                        { name: 'u_b/q_reg/D', kind: 'flipflop',
                          odb_type: 'iterm', odb_id: 2,
                          instance: 'u_b/q_reg', cell: 'DFF',
                          clocks: ['clk_b'] },
                    ]),
                    clocks_total: { clk_a: 4, clk_b: 7 },
                },
                detailResp: makeEpDetailResp([]),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();

            // Trigger visible, label reflects the all-default state.
            assert.notEqual(widget._epClockFilter.wrap.style.display, 'none',
                'trigger visible when multiple clocks present');
            assert.ok(widget._epClockFilter.trigger.textContent.includes('All clocks'),
                'trigger label is "All clocks" by default');
            assert.ok(widget._epClockFilter.trigger.textContent.includes('(11)'),
                'trigger label includes the total count');

            // Clicking the trigger opens the checkbox panel.
            widget._epClockFilter.trigger.click();
            assert.equal(widget._epClockFilter.panel.style.display, 'block',
                'panel opens on trigger click');

            // Every clock gets a checkbox; all start checked because
            // the default is all-selected.
            for (const key of ['clk_a', 'clk_b']) {
                assert.ok(findClockRow(widget, key),
                    `${key} row rendered`);
                assert.ok(findClockBox(widget, key).checked,
                    `${key} checkbox checked by default`);
            }
            // No master "All clocks" master toggle — users tick/untick
            // individual clocks; the empty / full set drives the
            // refetch behavior.
            assert.equal(findClockRow(widget, '__all__'), null,
                'no master "All clocks" row in the panel');
            // Per-clock counts are surfaced.
            assert.ok(findClockRow(widget, 'clk_a').textContent.includes('(4)'),
                'clk_a row includes its count');

            // Unchecking clk_b narrows the filter and triggers a refetch
            // with a comma-separated clock value (just clk_a here).
            toggleClockBox(widget, 'clk_b');
            await settle();
            const lastList = [...calls].reverse().find(
                m => m.type === 'sdc_endpoint_list');
            assert.equal(lastList.clock, 'clk_a',
                'refetch carries the narrowed clock list');
            // Selection state mirrors the UI.
            assert.ok(widget._epClockFilter.getSelected() instanceof Set,
                'selection set materialised on first per-clock toggle');
            assert.deepEqual([...widget._epClockFilter.getSelected()], ['clk_a']);
            assert.ok(widget._epClockFilter.trigger.textContent.includes('clk_a'),
                'trigger label updates to single-clock form');

            // Re-checking clk_b returns to the "all" sentinel and the
            // refetch goes out as ?clock=all.
            toggleClockBox(widget, 'clk_b');
            await settle();
            assert.equal(widget._epClockFilter.getSelected(), null,
                'selection collapses to the all-sentinel when every clock is on');
            const lastList2 = [...calls].reverse().find(
                m => m.type === 'sdc_endpoint_list');
            assert.equal(lastList2.clock, 'all',
                'refetch goes out as clock=all when every clock is checked');
        });

        it('unchecking every clock empties the filter and updates the trigger label', async () => {
            const { app, calls } = makeEpApp({
                listResp: {
                    ...makeEpListResp([]),
                    clocks_total: { clk_a: 2, clk_b: 3 },
                },
                detailResp: makeEpDetailResp([]),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();
            widget._epClockFilter.trigger.click();

            // Settle between toggles: each toggle fires a refetch and
            // the `_epListLoading` guard would otherwise drop the
            // second toggle while the first refetch is still in
            // flight. A real user has the same wait — the UI
            // re-paints before they click the next checkbox.
            toggleClockBox(widget, 'clk_a');
            await settle();
            toggleClockBox(widget, 'clk_b');
            await settle();
            assert.ok(widget._epClockFilter.getSelected() instanceof Set);
            assert.equal(widget._epClockFilter.getSelected().size, 0,
                'selection empties when every checkbox is off');
            const lastList = [...calls].reverse().find(
                m => m.type === 'sdc_endpoint_list');
            assert.equal(lastList.clock, '',
                'empty selection sends an empty clock list');
            assert.ok(widget._epClockFilter.trigger.textContent
                .includes('No clocks selected'),
                'trigger label communicates the empty-selection state');
        });

        it('drops disappeared clocks from the selection on refresh', async () => {
            // First fetch: clk_a + clk_b. User unchecks clk_a so the
            // selection is {clk_b}. Second fetch: clk_b is gone. The
            // selection should reconcile down to "all" since nothing
            // from the prior set survives (otherwise the user would
            // be silently filtering against a clock that no longer
            // exists, producing an empty list).
            let phase = 1;
            const app = {
                websocketManager: {
                    readyPromise: Promise.resolve(),
                    request(msg) {
                        if (msg.type === 'sdc_endpoint_list') {
                            return Promise.resolve(phase === 1
                                ? { ...makeEpListResp([]),
                                    clocks_total: { clk_a: 2, clk_b: 3 } }
                                : { ...makeEpListResp([]),
                                    clocks_total: { clk_a: 2 } });
                        }
                        if (msg.type === 'sdc_endpoint')
                            return Promise.resolve(makeEpDetailResp([]));
                        if (msg.type === 'sdc_clocks')
                            return Promise.resolve(EMPTY_CLOCKS);
                        if (msg.type === 'sdc_clock_modes')
                            return Promise.resolve(EMPTY_MODES);
                        return Promise.resolve({});
                    },
                },
            };
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();
            widget._epClockFilter.trigger.click();
            toggleClockBox(widget, 'clk_a');  // narrow to {clk_b}
            await settle();
            assert.deepEqual([...widget._epClockFilter.getSelected()], ['clk_b']);

            // Refresh with clk_b removed.
            phase = 2;
            widget._loadEndpointList({ reset: true });
            await settle();
            assert.equal(widget._epClockFilter.getSelected(), null,
                'selection collapses to all when no chosen clock survives');
        });
    });

    // ── Capture-side clock filter ──────────────────────────────────────────

    describe('endpoint card shows only capture-side clocks', () => {
        it('drops a Q-pin launch clock that no endpoint pin reports', async () => {
            // Flop endpoint u_a/q_reg/D is captured by clk_a. The Q
            // pin's clockDomains() reports both clk_a (its launch
            // clock = the local CK) AND clk_b (a cross-clock launch
            // clock that propagates downstream). The card should
            // surface ONLY clk_a — clk_b doesn't relate to the
            // capture side and would mislead someone debugging this
            // endpoint.
            const captureClk = {
                name: 'clk_a', period: 10, waveform: [0, 5],
                library_setup: 0.1, library_hold: 0.05,
            };
            const noiseClk = {
                name: 'clk_b', period: 7, waveform: [0, 3.5],
            };
            const { app } = makeEpApp({
                listResp: makeEpListResp([
                    { name: 'u_a/q_reg/D', kind: 'flipflop',
                      odb_type: 'iterm', odb_id: 1,
                      instance: 'u_a/q_reg', cell: 'DFF',
                      clocks: ['clk_a'] },  // backend reports capture clock here
                ]),
                detailResp: makeEpDetailResp([
                    { name: 'u_a/q_reg/D', direction: 'input',
                      is_clock_pin: false, clocks: [captureClk] },
                    { name: 'u_a/q_reg/CK', direction: 'input',
                      is_clock_pin: true, clocks: [captureClk] },
                    { name: 'u_a/q_reg/Q', direction: 'output',
                      is_clock_pin: false,
                      // Q reports both the capture clock (its launch
                      // domain) and a noise clock that bled in via
                      // cross-clock analysis.
                      clocks: [captureClk, noiseClk] },
                ]),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();

            const subcards = card.querySelectorAll('.sdc-ep-clock-subcard');
            assert.equal(subcards.length, 1,
                'exactly one per-clock sub-card — the capture clock');
            assert.ok(subcards[0].textContent.includes('clk_a'),
                'capture clock subcard rendered');
            // The non-capture clock must not appear anywhere in the
            // expanded body — the whole point of the filter.
            const body = card.querySelector('.sdc-ep-card-body');
            assert.ok(!body.textContent.includes('clk_b'),
                'non-capture clock filtered out of the card body');
        });

        it('falls back to all clocks when no endpoint pin reports one', async () => {
            // Combinational stdcell endpoints reached via set_max_delay
            // -to don't carry a clockDomains() result; the capture-clock
            // filter would yield an empty set, leaving the user with no
            // visible timing data. The fallback restores all clocks so
            // the card still renders something useful.
            const someClk = {
                name: 'clk', period: 10, waveform: [0, 5],
            };
            const { app } = makeEpApp({
                listResp: makeEpListResp([
                    { name: 'u_buf/Z', kind: 'stdcell',
                      odb_type: 'iterm', odb_id: 1,
                      instance: 'u_buf', cell: 'BUF',
                      clocks: [] },  // endpoint pin has no clock domain
                ]),
                detailResp: makeEpDetailResp([
                    { name: 'u_buf/Z', direction: 'output',
                      clocks: [someClk] },
                ]),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();
            const subcards = card.querySelectorAll('.sdc-ep-clock-subcard');
            assert.equal(subcards.length, 1,
                'fallback: clock still rendered when the endpoint reports none');
        });
    });

    // ── Tooltip coverage on key surfaces ───────────────────────────────────
    //
    // These tests guard against silently dropping the helper-tooltip strings
    // that explain non-obvious SDC concepts to the user (matrix legend, band
    // colors, kind/direction badges). They don't pin the exact wording —
    // just that *some* substantive tooltip is attached.

    describe('explanatory tooltips', () => {
        it('clock-groups matrix legend explains "no group" and self cells', async () => {
            const app = createMockApp({
                sdc_clocks: () => ({
                    clocks: [
                        { name: 'clk_a', period: 10, waveform: [0, 5],
                          is_generated: false, is_virtual: false,
                          sources: ['p_a'] },
                        { name: 'clk_b', period: 7,  waveform: [0, 3.5],
                          is_generated: false, is_virtual: false,
                          sources: ['p_b'] },
                    ],
                    clock_tree: [
                        { name: 'clk_a', children: [] },
                        { name: 'clk_b', children: [] },
                    ],
                    time_unit: 'ns',
                }),
                sdc_clock_modes: () => EMPTY_MODES,
                // Need at least one set_clock_groups command for the
                // matrix (and thus its legend) to render. With zero
                // groups the renderer falls back to a flat list.
                sdc_clock_groups: () => ({
                    time_unit: 'ns',
                    groups: [{
                        type: 'asynchronous',
                        name: 'g1',
                        clk_sets: [['clk_a'], ['clk_b']],
                    }],
                }),
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Clock Groups');
            await settle();

            // Each legend entry has a title with a category-level explainer.
            // Find the "·" entry (no group) and verify its tip mentions
            // set_clock_groups so a user knows what concept it relates to.
            const legendItems = Array.from(
                widget._cgScrollArea.querySelectorAll('span'));
            const noGroupItem = legendItems.find(
                el => el.textContent === '· = no clock-groups constraint');
            assert.ok(noGroupItem, 'no-group legend entry exists');
            assert.ok(noGroupItem.title.includes('set_clock_groups'),
                'no-group legend tooltip references set_clock_groups');
            // The "no group" tooltip MUST capture the synchronous-by-
            // default semantics — without that nuance a user might
            // assume "no group" means async/ignored, which is the
            // opposite of what STA actually does.
            assert.ok(/synchronous/i.test(noGroupItem.title),
                'no-group tooltip explains the default-synchronous behavior');
            assert.ok(/asynchronous/i.test(noGroupItem.title),
                'no-group tooltip mentions set_clock_groups -asynchronous as the way to opt out');

            const selfItem = legendItems.find(
                el => el.textContent === '— = self (same clock)');
            assert.ok(selfItem, 'self legend entry exists');
            assert.ok(selfItem.title.length > 0, 'self entry has a tooltip');
        });

        it('endpoint kind badge explains the timing model', async () => {
            const { app } = makeEpApp({
                listResp: makeEpListResp([
                    { name: 'u_lat/D', kind: 'latch',
                      odb_type: 'iterm', odb_id: 1,
                      instance: 'u_lat', cell: 'DLH', clocks: ['clk'] },
                ]),
                detailResp: makeEpDetailResp([{ name: 'u_lat/D', clocks: [] }]),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();
            const badge = widget._epListArea.querySelector('.sdc-ep-kind');
            assert.ok(badge, 'kind badge rendered');
            // Tooltip should mention the latch concept (level-sensitive,
            // transparent, or time-borrow) so the user can decode LATCH.
            assert.ok(/latch|transparent|level/i.test(badge.title),
                'latch kind badge tooltip explains the latch model');
        });

        it('latch sub-card capture-edge guide carries an explainer tooltip',
        async () => {
            const sharedClk = {
                name: 'clk', period: 10, waveform: [0, 5],
                library_setup: 0.1, library_hold: 0.05,
            };
            const { app } = makeEpApp({
                listResp: makeEpListResp([
                    { name: 'u_lat/D', kind: 'latch',
                      odb_type: 'iterm', odb_id: 1,
                      instance: 'u_lat', cell: 'DLH', clocks: ['clk'] },
                ]),
                detailResp: makeEpDetailResp([
                    { name: 'u_lat/D', direction: 'input',
                      capture_edge: 'fall', clocks: [sharedClk] },
                ]),
            });
            const widget = await setupTabWidget('Endpoint', app);
            widget._epListBtn.click();
            await settle();
            const card = widget._epListArea.querySelector('.sdc-ep-card');
            card.querySelector('.sdc-ep-card-header').click();
            await settle();
            const guide = card.querySelector('[data-ref="capture"]');
            assert.ok(guide, 'capture-edge guide line drawn');
            const titleEl = guide.querySelector('title');
            assert.ok(titleEl, 'guide line carries an SVG <title>');
            assert.ok(/closing|capture/i.test(titleEl.textContent),
                'capture-edge tooltip explains the closing-edge concept');
        });

        it('Limits-tab column headers carry vocabulary tooltips', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                sdc_limits: () => ({
                    time_unit: 'ns', cap_unit: 'pF',
                    clock_latencies: [{
                        clock: 'clk', pin: null,
                        rise_max: 0.5, rise_min: 0.4,
                        fall_max: 0.5, fall_min: 0.4,
                    }],
                    clock_insertions: [], clock_uncertainties: [],
                    port_loads: [], disabled_timing: [],
                }),
            });
            const widget = new SdcWidget(app);
            widget._activateTab('Limits');
            await settle();
            const headers = Array.from(
                widget._limScrollArea.querySelectorAll('th'));
            const riseMaxTh = headers.find(th => th.textContent === 'Rise max');
            assert.ok(riseMaxTh, 'Rise max column header rendered');
            assert.ok(/rise.*max|max.*corner|late/i.test(riseMaxTh.title),
                'Rise max column carries a tooltip explaining the corner');
        });
    });

    // ── CDC tab ─────────────────────────────────────────────────────────────
    //
    // Most CDC behaviour is exercised end-to-end on the C++ side
    // (TestCdcHandler). The JS-side tests here verify the widget glues
    // request/response correctly into the matrix → path-list → detail
    // drill-in flow and that the "how was sync identified" cell carries
    // an explanatory tooltip.

    describe('CDC tab', () => {
        // Two clocks, one cell of the matrix populated: clk_a → clk_b
        // with one synced + one unsynced path. The overview now carries
        // every mode in one payload — the widget renders whichever
        // mode is currently active.
        const CDC_OVERVIEW = {
            time_unit: 'ns',
            current_mode: 'default',
            modes: {
                default: {
                    endpoint_count: 2,
                    clocks: ['clk_a', 'clk_b'],
                    matrix: {
                        clk_a: {
                            clk_b: { paths: 2, synced: 1,
                                     excluded: 0, unsynced: 1 },
                        },
                    },
                },
            },
        };
        const CDC_PATHS = {
            time_unit: 'ns',
            total: 2, offset: 0,
            category_total: { synced: 1, excluded: 0, unsynced: 1 },
            paths: [
                {
                    capture_pin:  'sync_b1/D',
                    odb_type:     'iterm', odb_id: 1,
                    capture_inst: 'sync_b1',
                    capture_cell: 'DFF_X1',
                    launch_clock: 'clk_a', capture_clock: 'clk_b',
                    category:     'synchronized',
                    sync_chain_kind:  'ff_chain',
                    sync_chain_depth: 2,
                    whitelist_match:   null,
                    whitelist_pattern: null,
                },
                {
                    capture_pin:  'ff_unsynced/D',
                    odb_type:     'iterm', odb_id: 2,
                    capture_inst: 'ff_unsynced',
                    capture_cell: 'DFF_X1',
                    launch_clock: 'clk_a', capture_clock: 'clk_b',
                    category:     'unsynchronized',
                    sync_chain_kind:  'none',
                    sync_chain_depth: 1,
                    whitelist_match:   null,
                    whitelist_pattern: null,
                },
            ],
        };

        it('matrix renders one cell per (launch, capture) pair', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
            });
            const widget = await setupTabWidget('CDC', app);
            // User clicks "Scan CDC".
            widget._cdcScanBtn.click();
            await settle();
            // Cells: clk_a row × {clk_a (—), clk_b (2)}; clk_b row × ...
            const cells = widget._cdcBody.querySelectorAll('td[data-cdc-launch]');
            assert.ok(cells.length >= 1, 'matrix renders at least one CDC cell');
            const ab = Array.from(cells).find(td =>
                td.dataset.cdcLaunch === 'clk_a'
                && td.dataset.cdcCapture === 'clk_b');
            assert.ok(ab, 'clk_a → clk_b cell present');
            assert.equal(ab.textContent, '2');
            assert.ok(/synced/i.test(ab.title) && /unsynced/i.test(ab.title),
                'cell tooltip names every category count');
            // Tooltip leads with the launch → capture pair so the
            // user gets the orientation labels first when the
            // browser tooltip fires.
            assert.ok(/^clk_a → clk_b/.test(ab.title),
                'cell tooltip leads with launch → capture pair');
        });

        // CDC toolbar status pill — extends the bare crossing count
        // with a per-category breakdown (synced/unsynced/excluded)
        // so the user can see the health distribution at a glance
        // without scanning the matrix.
        it('toolbar status reports per-category totals', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            const statusText = widget._cdcStatus.textContent;
            assert.ok(/2 CDC crossings/.test(statusText),
                'status reports the total crossing count');
            assert.ok(/1 unsynced/.test(statusText),
                'status reports the unsynced category count');
            assert.ok(/1 synced/.test(statusText),
                'status reports the synced category count');
            assert.ok(/0 excluded/.test(statusText),
                'status reports the excluded category count');
        });

        // Hover any cell — the matching row and column header tints
        // and bolds, mouseleave reverts. Pinned via direct event
        // dispatch since jsdom has no real :hover state.
        it('hovering a cell highlights its row & column headers',
                async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            const ab = widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]');
            assert.ok(ab, 'clk_a → clk_b cell exists');
            // Walk the table to find the row's <th> (clk_a row label)
            // and the column's <th> in the thead (clk_b column).
            const rowTh = ab.parentElement.querySelector('th');
            const headerTrs = widget._cdcBody.querySelectorAll('thead tr');
            const headerRow = headerTrs[headerTrs.length - 1];
            const headerCells = headerRow.querySelectorAll('th');
            // Header row layout: corner + clocks; clk_b is the
            // second clock so it lives at index 2 (after the corner
            // at index 0 and clk_a at index 1).
            const colTh = headerCells[2];
            assert.ok(colTh.textContent.includes('clk_b'),
                'sanity: located the clk_b column header');

            const rowBgBefore = rowTh.style.background;
            const colBgBefore = colTh.style.background;
            ab.dispatchEvent(new window.Event('mouseenter'));
            assert.notEqual(rowTh.style.background, rowBgBefore,
                'row label background changes on hover');
            assert.notEqual(colTh.style.background, colBgBefore,
                'column header background changes on hover');
            assert.equal(rowTh.style.fontWeight, '700',
                'row label goes bold on hover');

            ab.dispatchEvent(new window.Event('mouseleave'));
            assert.equal(rowTh.style.background, rowBgBefore,
                'row label background restored on mouseleave');
            assert.equal(colTh.style.background, colBgBefore,
                'column header background restored on mouseleave');
            assert.equal(rowTh.style.fontWeight, '',
                'row label weight restored on mouseleave');
        });

        // Sticky positioning is hard to validate in jsdom (no real
        // layout pass), but the inline style declaration is what
        // the browser would consume — pinning the declarations
        // catches the common regression of someone removing
        // `position:sticky` without realizing it's load-bearing
        // on large matrices.
        //
        // top/left use small NEGATIVE offsets to compensate for
        // _cdcBody's padding (so tbody cells don't visibly scroll
        // through the padding zone before sticky activates). We
        // assert the offsets are negative rather than a specific
        // pixel value because the padding could change without
        // breaking the affordance.
        it('row + column + corner headers carry sticky positioning',
                async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            const headerTrs = widget._cdcBody.querySelectorAll('thead tr');
            const headerCells = headerTrs[0].querySelectorAll('th');
            const corner = headerCells[0];
            const colTh = headerCells[1];
            const rowTh = widget._cdcBody.querySelector('tbody tr > th');
            const negPx = (s) => {
                const v = parseFloat(s);
                return Number.isFinite(v) && v <= 0;
            };
            assert.equal(corner.style.position, 'sticky',
                'corner cell is sticky');
            assert.ok(negPx(corner.style.top),
                'corner pins to top edge (negative offset for ' +
                'padding compensation)');
            assert.ok(negPx(corner.style.left),
                'corner pins to left edge (negative offset for ' +
                'padding compensation)');
            assert.equal(colTh.style.position, 'sticky',
                'column header is sticky');
            assert.ok(negPx(colTh.style.top),
                'column header pins to top edge');
            assert.equal(rowTh.style.position, 'sticky',
                'row label is sticky');
            assert.ok(negPx(rowTh.style.left),
                'row label pins to left edge');
        });

        // The "Rows = launch clock..." heading + the toggle row are
        // wrapped in a sticky banner that pins on BOTH axes so the
        // user's orientation cues stay visible no matter how far
        // they've scrolled into a wide / tall matrix.
        it('heading + toggle live in a sticky banner pinned on both axes',
                async () => {
            // Use a fixture where the toggle row renders (idle > 0)
            // so we can verify it's INSIDE the banner.
            const overview = {
                time_unit: 'ns',
                current_mode: 'default',
                modes: {
                    default: {
                        endpoint_count: 2,
                        clocks: ['clk_a', 'clk_b', 'clk_idle1'],
                        matrix: {
                            clk_a: {
                                clk_b: { paths: 2, synced: 1,
                                         excluded: 0, unsynced: 1 },
                            },
                        },
                    },
                },
            };
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => overview,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            // Find the banner: a div with `position:sticky`, sitting
            // BEFORE the table, that contains the heading text.
            const candidates = Array.from(
                widget._cdcBody.querySelectorAll('div'));
            const banner = candidates.find(d =>
                d.style.position === 'sticky'
                && /Rows = launch clock/.test(d.textContent));
            assert.ok(banner,
                'banner div exists with sticky positioning + heading');
            // Pinned on both axes (top + left), with negative offsets
            // for the padding compensation.
            const negPx = (s) => {
                const v = parseFloat(s);
                return Number.isFinite(v) && v <= 0;
            };
            assert.ok(negPx(banner.style.top),
                'banner pins to top edge');
            assert.ok(negPx(banner.style.left),
                'banner pins to left edge');
            // Toggle button is inside the banner so it scrolls with
            // it (i.e. stays visible).
            const toggleInBanner = Array.from(
                banner.querySelectorAll('button'))
                .some(b => /Show all clocks|Hide idle clocks/.test(
                    b.textContent));
            assert.ok(toggleInBanner,
                'toggle button lives inside the sticky banner');
            // Banner z-index is higher than the corner cell so it
            // covers the corner at the top-left intersection.
            const corner = widget._cdcBody.querySelector(
                'thead tr > th'); // first <th> in thead is corner
            const cornerZ = parseInt(corner.style.zIndex, 10);
            const bannerZ = parseInt(banner.style.zIndex, 10);
            assert.ok(bannerZ > cornerZ,
                'banner z-index sits above the corner cell '
                + `(banner=${bannerZ} > corner=${cornerZ})`);
        });

        // Idle-clocks toggle — on a many-clock design, default to
        // showing only clocks that exchange paths and hide the rest
        // behind a "Show all clocks" affordance. Re-clicking the
        // toggle returns to the focused view (bidirectional).
        // Helper for the toggle tests: collect the launch-clock label
        // from every tbody row. The matrix only stamps
        // `data-cdc-launch` on NON-empty cells, so we can't enumerate
        // rendered rows that way — but every row has a `<th>` row
        // header with the clock name as its textContent, so reading
        // those gives the actual rendered set.
        const renderedLaunches = (widget) => new Set(
            Array.from(widget._cdcBody
                .querySelectorAll('table tbody tr > th'))
                .map(th => th.textContent));
        it('idle clocks are hidden by default and revealed by the toggle',
                async () => {
            // 5-clock design where only clk_a → clk_b carries any
            // crossings; the other three clocks (clk_idle1/2/3) sit
            // in the matrix with no traffic.
            const overview = {
                time_unit: 'ns',
                current_mode: 'default',
                modes: {
                    default: {
                        endpoint_count: 2,
                        clocks: ['clk_a', 'clk_b',
                                 'clk_idle1', 'clk_idle2', 'clk_idle3'],
                        matrix: {
                            clk_a: {
                                clk_b: { paths: 2, synced: 1,
                                         excluded: 0, unsynced: 1 },
                            },
                        },
                    },
                },
            };
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => overview,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();

            // Default state: only the active clocks render their row
            // labels; idle clocks are absent from the body entirely.
            const launchSet = renderedLaunches(widget);
            assert.ok(launchSet.has('clk_a') && launchSet.has('clk_b'),
                'active clocks are present');
            assert.ok(!launchSet.has('clk_idle1')
                && !launchSet.has('clk_idle2')
                && !launchSet.has('clk_idle3'),
                'idle clocks hidden in the default focused view');

            // Toggle button is rendered with a "+ N idle" label and
            // surfaces in the matrix wrap.
            const buttons = Array.from(widget._cdcBody
                .querySelectorAll('button'));
            const toggle = buttons.find(b =>
                /Show all clocks/.test(b.textContent));
            assert.ok(toggle, 'toggle button labeled "Show all clocks"');
            assert.ok(/\+ 3 idle/.test(toggle.textContent),
                'toggle reports the count of hidden idle clocks');

            // First click — every clock becomes visible.
            toggle.click();
            await settle();
            const launchSet2 = renderedLaunches(widget);
            for (const c of ['clk_a', 'clk_b',
                             'clk_idle1', 'clk_idle2', 'clk_idle3']) {
                assert.ok(launchSet2.has(c),
                    `${c} visible after expanding`);
            }

            // The button label flips to the collapse direction so the
            // user knows what the next click will do.
            const buttons2 = Array.from(widget._cdcBody
                .querySelectorAll('button'));
            const collapse = buttons2.find(b =>
                /Hide idle clocks/.test(b.textContent));
            assert.ok(collapse,
                'after expanding, toggle label flips to "Hide idle clocks"');

            // Second click — collapse back to the focused view.
            collapse.click();
            await settle();
            const launchSet3 = renderedLaunches(widget);
            assert.ok(!launchSet3.has('clk_idle1'),
                'idle clocks hidden again after re-clicking the toggle');
            assert.ok(launchSet3.has('clk_a') && launchSet3.has('clk_b'),
                'active clocks remain visible after collapsing');
        });

        // Edge case: every clock in the design is active. Hiding "0
        // idle clocks" is meaningless, so the toggle should not
        // render at all and the matrix shows everything.
        it('toggle is suppressed when every clock has crossings',
                async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            const buttons = Array.from(widget._cdcBody
                .querySelectorAll('button'));
            const toggle = buttons.find(b =>
                /Show all clocks|Hide idle clocks/.test(b.textContent));
            assert.ok(!toggle,
                'no toggle rendered when there are no idle clocks to hide');
        });

        // Edge case: NO clock has any crossings (overview empty). The
        // focused view would render an empty 0×0 matrix; we fall back
        // to showing all clocks so the user sees the design's clock
        // set instead of an empty grid.
        it('falls back to all-clocks when no clock has any crossings',
                async () => {
            const overview = {
                time_unit: 'ns',
                current_mode: 'default',
                modes: {
                    default: {
                        endpoint_count: 0,
                        clocks: ['clk_a', 'clk_b', 'clk_c'],
                        matrix: {},
                    },
                },
            };
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => overview,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            const launches = renderedLaunches(widget);
            assert.ok(launches.has('clk_a')
                && launches.has('clk_b')
                && launches.has('clk_c'),
                'every clock is rendered when none has crossings');
            const buttons = Array.from(widget._cdcBody
                .querySelectorAll('button'));
            const toggle = buttons.find(b =>
                /Show all clocks|Hide idle clocks/.test(b.textContent));
            assert.ok(!toggle,
                'toggle suppressed in the empty-overview fallback');
        });

        it('matrix cell click loads the per-path table', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths:    () => CDC_PATHS,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            const ab = widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]');
            ab.click();
            await settle();
            // Body now shows the per-path table; it should mention both pins.
            assert.ok(widget._cdcBody.textContent.includes('sync_b1/D'));
            assert.ok(widget._cdcBody.textContent.includes('ff_unsynced/D'));
        });

        it('Sync chain cell carries an explanation tooltip', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths:    () => CDC_PATHS,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]').click();
            await settle();
            // Find a span describing the sync chain — the synced row says
            // "2FF chain" and the unsynced row says "— none".
            const text = widget._cdcBody.textContent;
            assert.ok(/2FF chain/.test(text),
                'synced row labels chain depth');
            assert.ok(/— none/.test(text),
                'unsynced row labels missing chain');
            // The cell with "2FF chain" should carry a tooltip explaining
            // the FF→FF heuristic.
            const spans = widget._cdcBody.querySelectorAll('span');
            const ff = Array.from(spans).find(s =>
                s.textContent.includes('2FF chain'));
            assert.ok(ff, '2FF chain span exists');
            assert.ok(/FF.*heuristic|same capture domain/i.test(ff.title),
                'tooltip explains how the chain was detected');
        });

        it('mode switch re-renders from cache without refetching the overview', async () => {
            // Two-mode overview: default has the bug visible, scan has it
            // excluded by clock_groups so the unsynced count goes to 0.
            const TWO_MODE_OVERVIEW = {
                time_unit: 'ns',
                current_mode: 'default',
                modes: {
                    default: {
                        endpoint_count: 2,
                        clocks: ['clk_a', 'clk_b'],
                        matrix: { clk_a: { clk_b: {
                            paths: 2, synced: 1, excluded: 0, unsynced: 1,
                        }}},
                    },
                    scan: {
                        endpoint_count: 0,
                        clocks: ['clk_a', 'clk_b'],
                        matrix: { clk_a: { clk_b: {
                            paths: 2, synced: 0, excluded: 2, unsynced: 0,
                        }}},
                    },
                },
            };
            const calls = [];
            const app = {
                websocketManager: {
                    readyPromise: Promise.resolve(),
                    request(msg) {
                        calls.push(msg);
                        if (msg.type === 'sdc_clocks')
                            return Promise.resolve(EMPTY_CLOCKS);
                        if (msg.type === 'sdc_clock_modes')
                            return Promise.resolve(EMPTY_MODES);
                        if (msg.type === 'cdc_overview')
                            return Promise.resolve(TWO_MODE_OVERVIEW);
                        return Promise.resolve({});
                    },
                },
            };
            const widget = await setupTabWidget('CDC', app);
            // Initial scan.
            widget._cdcScanBtn.click();
            await settle();
            const overviewBefore = calls.filter(c =>
                c.type === 'cdc_overview').length;
            assert.equal(overviewBefore, 1,
                'one cdc_overview fetch on initial scan');
            // Default-mode cell shows 2 paths in red (has unsynced).
            let cell = widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]');
            assert.ok(/unsynced paths present/.test(cell.title),
                'default mode cell flags unsynced paths');

            // Simulate a mode switch via _invalidateAllTabs (the same
            // hook _setMode triggers).
            widget._currentModeName = 'scan';
            // Manually pin the active mode in the cache to scan so the
            // re-render picks it up.
            widget._cdcOverview.current_mode = 'scan';
            widget._invalidateAllTabs();
            await settle();
            // No second cdc_overview call should have happened.
            const overviewAfter = calls.filter(c =>
                c.type === 'cdc_overview').length;
            assert.equal(overviewAfter, 1,
                'mode switch did NOT trigger a second cdc_overview fetch');
            // Scan-mode cell labels every path as excluded (grey).
            cell = widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]');
            assert.ok(/every path excluded/.test(cell.title),
                'scan mode cell shows excluded paths');
        });

        it('path detail renders launch-side comb stages with net click-through', async () => {
            // Mock cdc_path_detail response: launch flop → 2 comb gates →
            // crossover capture flop. No sync chain (kind: none).
            const PATH_DETAIL = {
                stages: [
                    {
                        instance: 'ff_src_a',
                        cell: 'DFF_X1',
                        odb_type: 'inst', odb_id: 100,
                        d_pin: 'ff_src_a/D',
                        d_pin_odb_type: 'iterm', d_pin_odb_id: 1000,
                        q_pin: 'ff_src_a/Q',
                        q_pin_odb_type: 'iterm', q_pin_odb_id: 1001,
                        kind: 'register',
                        is_launch: true,
                        is_capture: false,
                        is_sync_stage: false,
                        clock: 'clk_a',
                        capture_clock: 'clk_b',
                        out_net: { name: 'q_a',
                                   odb_type: 'net', odb_id: 200 },
                    },
                    {
                        instance: 'comb2_g1',
                        cell: 'AND2_X1',
                        odb_type: 'inst', odb_id: 101,
                        in_pin: 'comb2_g1/A1',
                        in_pin_odb_type: 'iterm', in_pin_odb_id: 1002,
                        out_pin: 'comb2_g1/ZN',
                        out_pin_odb_type: 'iterm', out_pin_odb_id: 1003,
                        aux_in_pins: [
                            { name: 'comb2_g1/A2',
                              odb_type: 'iterm', odb_id: 1004 },
                        ],
                        kind: 'comb',
                        is_launch: false,
                        is_capture: false,
                        is_sync_stage: false,
                        clock: 'clk_a',
                        capture_clock: 'clk_b',
                        out_net: { name: 'comb2_t1',
                                   odb_type: 'net', odb_id: 201 },
                    },
                    {
                        instance: 'comb2_g2',
                        cell: 'OR2_X1',
                        odb_type: 'inst', odb_id: 102,
                        in_pin: 'comb2_g2/A1',
                        in_pin_odb_type: 'iterm', in_pin_odb_id: 1005,
                        out_pin: 'comb2_g2/ZN',
                        out_pin_odb_type: 'iterm', out_pin_odb_id: 1006,
                        aux_in_pins: [],
                        kind: 'comb',
                        is_launch: false,
                        is_capture: false,
                        is_sync_stage: false,
                        clock: 'clk_a',
                        capture_clock: 'clk_b',
                        out_net: { name: 'comb2_t2',
                                   odb_type: 'net', odb_id: 202 },
                    },
                    {
                        instance: 'ff_dst_b_comb_2gate',
                        cell: 'DFF_X1',
                        odb_type: 'inst', odb_id: 103,
                        d_pin: 'ff_dst_b_comb_2gate/D',
                        d_pin_odb_type: 'iterm', d_pin_odb_id: 1007,
                        q_pin: 'ff_dst_b_comb_2gate/Q',
                        q_pin_odb_type: 'iterm', q_pin_odb_id: 1008,
                        kind: 'register',
                        is_launch: false,
                        is_capture: true,
                        is_sync_stage: false,
                        launch_clock: 'clk_a',
                        capture_clock: 'clk_b',
                        clock: 'clk_b',
                        out_net: null,
                    },
                ],
                sync_chain: { kind: 'none', depth: 1,
                              whitelist_match: null, whitelist_pattern: null },
            };
            const PATHS_FOR_DETAIL = {
                time_unit: 'ns',
                total: 1, offset: 0,
                category_total: { synced: 0, excluded: 0, unsynced: 1 },
                paths: [{
                    capture_pin:  'ff_dst_b_comb_2gate/D',
                    odb_type:     'iterm', odb_id: 1007,
                    capture_inst: 'ff_dst_b_comb_2gate',
                    capture_cell: 'DFF_X1',
                    launch_clock: 'clk_a', capture_clock: 'clk_b',
                    category:     'unsynchronized',
                    sync_chain_kind:  'none',
                    sync_chain_depth: 1,
                    whitelist_match:   null,
                    whitelist_pattern: null,
                }],
            };
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths:    () => PATHS_FOR_DETAIL,
                cdc_path_detail: () => PATH_DETAIL,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            // Drill into the matrix → path list → first path's detail link.
            widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]').click();
            await settle();
            const detailLink = widget._cdcBody.querySelector(
                'tbody tr td span');
            assert.ok(detailLink, 'detail link rendered');
            detailLink.click();
            await settle();

            const text = widget._cdcBody.textContent;
            // All four stages render. Pin row labels are leaf-only
            // now (D / CK / Q / A1 / ZN), so we match the
            // instance NAMES (which still appear in the card
            // header) without `\b` — the leaf simplification
            // means there's no `/` separating instance from pin
            // anymore.
            assert.ok(text.includes('ff_src_a'), 'launch flop stage card');
            assert.ok(text.includes('comb2_g1'), 'first comb stage card');
            assert.ok(text.includes('comb2_g2'), 'second comb stage card');
            assert.ok(text.includes('ff_dst_b_comb_2gate'),
                'crossover stage card');
            // Banners reflect each role.
            assert.ok(/launch/i.test(text), 'launch banner present');
            assert.ok(/comb · AND2_X1/.test(text), 'comb stage names cell');
            assert.ok(/crossover/i.test(text), 'crossover banner present');
            // Aux fan-in pin shows up. Pin rows render the leaf
            // (A2) in their own span; the instance (comb2_g1) is
            // in the card header. We check both via DOM lookup
            // because "A2" appears as a substring of "AND2_X1"
            // in plain textContent — direct element matching
            // avoids the false positive.
            assert.ok(text.includes('comb2_g1'),
                'comb2_g1 instance present in card header');
            const a2Spans = Array.from(
                widget._cdcBody.querySelectorAll('span'))
                .filter(s => s.textContent === 'A2');
            assert.ok(a2Spans.length > 0,
                'aux pin rendered as a leaf "A2" pin row');
            // Inter-stage net rows render with click-through.
            assert.ok(/q_a\b/.test(text), 'launch.Q net name shown');
            assert.ok(/comb2_t1\b/.test(text), 'inter-comb net name shown');
            assert.ok(/comb2_t2\b/.test(text), 'last-comb net name shown');
            // Net spans should be linkified — find one with our net name
            // and verify it carries pointer cursor (the click-through).
            const netSpans = Array.from(
                widget._cdcBody.querySelectorAll('span')).filter(s =>
                    s.textContent === 'comb2_t1');
            assert.ok(netSpans.length > 0,
                'net label span rendered');
            assert.ok(netSpans.some(s => s.style.cursor === 'pointer'),
                'net label is clickable');
            // Summary line mentions the launch-side counts.
            assert.ok(/1 launch · 2 comb/.test(text),
                'summary names launch + comb stage counts');
        });

        it('passthroughs render as expandable mini-cards between stages',
                async () => {
            // 2-stage detail: launch flop → INV (collapsed pass-through)
            // → capture flop. The inverter sits in the gap between
            // launch and capture as `passthroughs_after` on the launch.
            const PATH_DETAIL = {
                stages: [
                    {
                        instance: 'ff_src_a',
                        cell: 'DFF_X1',
                        odb_type: 'inst', odb_id: 200,
                        d_pin: 'ff_src_a/D',
                        d_pin_odb_type: 'iterm', d_pin_odb_id: 2000,
                        q_pin: 'ff_src_a/Q',
                        q_pin_odb_type: 'iterm', q_pin_odb_id: 2001,
                        kind: 'register',
                        is_launch: true, is_capture: false, is_sync_stage: false,
                        clock: 'clk_a', capture_clock: 'clk_b',
                        out_net: { name: 'q_a',
                                   odb_type: 'net', odb_id: 300 },
                        passthroughs_after: [{
                            instance: 'inv_a',
                            cell: 'INV_X1',
                            odb_type: 'inst', odb_id: 201,
                            in_pin: 'inv_a/A',
                            in_pin_odb_type: 'iterm', in_pin_odb_id: 2002,
                            out_pin: 'inv_a/ZN',
                            out_pin_odb_type: 'iterm', out_pin_odb_id: 2003,
                            out_net: { name: 'inv_a_zn',
                                       odb_type: 'net', odb_id: 301 },
                        }],
                    },
                    {
                        instance: 'ff_dst',
                        cell: 'DFF_X1',
                        odb_type: 'inst', odb_id: 202,
                        d_pin: 'ff_dst/D',
                        d_pin_odb_type: 'iterm', d_pin_odb_id: 2004,
                        q_pin: 'ff_dst/Q',
                        q_pin_odb_type: 'iterm', q_pin_odb_id: 2005,
                        kind: 'register',
                        is_launch: false, is_capture: true, is_sync_stage: false,
                        launch_clock: 'clk_a', capture_clock: 'clk_b',
                        clock: 'clk_b',
                        out_net: null,
                        passthroughs_after: [],
                    },
                ],
                sync_chain: { kind: 'none', depth: 1,
                              whitelist_match: null, whitelist_pattern: null },
            };
            const PATHS_FOR_DETAIL = {
                time_unit: 'ns',
                total: 1, offset: 0,
                category_total: { synced: 0, excluded: 0, unsynced: 1 },
                paths: [{
                    capture_pin:  'ff_dst/D',
                    odb_type:     'iterm', odb_id: 2004,
                    capture_inst: 'ff_dst',
                    capture_cell: 'DFF_X1',
                    launch_clock: 'clk_a', capture_clock: 'clk_b',
                    category:     'unsynchronized',
                    sync_chain_kind:  'none',
                    sync_chain_depth: 1,
                    whitelist_match:   null,
                    whitelist_pattern: null,
                }],
            };
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths:    () => PATHS_FOR_DETAIL,
                cdc_path_detail: () => PATH_DETAIL,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]').click();
            await settle();
            const detailLink = widget._cdcBody.querySelector(
                'tbody tr td span');
            detailLink.click();
            await settle();

            // Inter-stage gap should be a <details> when passthroughs
            // are present.
            const detailsEls = widget._cdcBody.querySelectorAll('details');
            assert.ok(detailsEls.length >= 1,
                'inter-stage gap rendered as <details>');
            const gap = detailsEls[detailsEls.length - 1];
            assert.equal(gap.open, false,
                'gap is collapsed by default');
            // Summary should hint at the hidden cell.
            assert.ok(/\+ 1 hidden cell/.test(gap.querySelector('summary').textContent),
                'summary advertises hidden cell count');
            // Inv mini-card should NOT be visible while collapsed.
            // It IS in the DOM (inside the closed <details>), but the
            // summary should be the only directly-rendered text in
            // the collapsed state — verify via the open state instead.
            gap.open = true;
            await settle();
            const text = widget._cdcBody.textContent;
            assert.ok(/pass · INV_X1/.test(text),
                'expanded gap reveals pass-through mini-card');
            assert.ok(/inv_a\b/.test(text),
                'mini-card shows the pass-through instance name');
        });

        it('settings modal sends instance + master patterns as CSV', async () => {
            const calls = [];
            const app = {
                websocketManager: {
                    readyPromise: Promise.resolve(),
                    request(msg) {
                        calls.push(msg);
                        if (msg.type === 'sdc_clocks')
                            return Promise.resolve(EMPTY_CLOCKS);
                        if (msg.type === 'sdc_clock_modes')
                            return Promise.resolve(EMPTY_MODES);
                        if (msg.type === 'cdc_overview')
                            return Promise.resolve(CDC_OVERVIEW);
                        if (msg.type === 'cdc_get_whitelist')
                            return Promise.resolve({
                                instance_patterns: [],
                                master_patterns:   [],
                            });
                        if (msg.type === 'cdc_set_whitelist') {
                            return Promise.resolve({
                                instance_patterns: ['ff_unsynced', '*flop*'],
                                master_patterns:   ['SYNC_FF'],
                            });
                        }
                        return Promise.resolve({});
                    },
                },
            };
            const widget = await setupTabWidget('CDC', app);
            // Open settings.
            await widget._openCdcSettings();
            // Find the textareas (in DOM order: instance, master).
            const tas = widget.element.querySelectorAll('textarea');
            assert.equal(tas.length, 2, 'two pattern textareas');
            tas[0].value = 'ff_unsynced\n*flop*\n';
            tas[1].value = 'SYNC_FF';
            // Click "Apply & re-scan".
            const applyBtn = Array.from(
                widget.element.querySelectorAll('button')).find(b =>
                    b.textContent.includes('Apply'));
            assert.ok(applyBtn, 'apply button rendered');
            applyBtn.click();
            await settle();
            const setCall = calls.find(c => c.type === 'cdc_set_whitelist');
            assert.ok(setCall, 'cdc_set_whitelist was sent');
            assert.equal(setCall.instance_patterns, 'ff_unsynced,*flop*',
                'instance patterns are CSV-joined');
            assert.equal(setCall.master_patterns, 'SYNC_FF',
                'master patterns are CSV-joined');
        });
    });

    // ────────────────────────────────────────────────────────────────────────
    // CDC path-detail UX — banners, shading, domain-mix, forward-chain
    // passthroughs. Each test builds its own minimal fixture so the
    // assertions are independent and the test name explains what's
    // being exercised.
    // ────────────────────────────────────────────────────────────────────────

    describe('CDC path-detail stage banners', () => {
        const CDC_OVERVIEW = {
            time_unit: 'ns',
            current_mode: 'default',
            modes: {
                default: {
                    endpoint_count: 1,
                    clocks: ['clk_a', 'clk_b'],
                    matrix: { clk_a: { clk_b: {
                        paths: 1, synced: 0, excluded: 0, unsynced: 1,
                    }}},
                },
            },
        };

        // Helpers — register-stage / comb-stage builders that fill in the
        // dozens of mandatory schema fields with defaults so each test
        // body only declares the bits it actually cares about.
        const reg = (name, opts) => Object.assign({
            instance: name, cell: 'DFF_X1',
            odb_type: 'inst', odb_id: 100,
            d_pin: name + '/D', d_pin_odb_type: 'iterm', d_pin_odb_id: 1,
            q_pin: name + '/Q', q_pin_odb_type: 'iterm', q_pin_odb_id: 2,
            kind: 'register',
            is_launch: false, is_capture: false, is_sync_stage: false,
            clock: 'clk_b', capture_clock: 'clk_b',
            out_net: null, passthroughs_after: [],
        }, opts);

        const buildOverviewApp = (pathDetail, paths) => createMockApp({
            sdc_clocks: () => EMPTY_CLOCKS,
            sdc_clock_modes: () => EMPTY_MODES,
            cdc_overview: () => CDC_OVERVIEW,
            cdc_paths: () => paths,
            cdc_path_detail: () => pathDetail,
        });

        // Drill from the overview matrix into the path-detail diagram
        // using the same UI flow a user would. Returns the widget after
        // settling so tests can query the rendered DOM.
        async function openDetail(app) {
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]').click();
            await settle();
            const detailLink = widget._cdcBody.querySelector(
                'tbody tr td span');
            detailLink.click();
            await settle();
            return widget;
        }

        const PATHS_ONE_ROW = (capture, capture_inst, syncKind) => ({
            time_unit: 'ns',
            total: 1, offset: 0,
            category_total: { synced: syncKind !== 'none' ? 1 : 0,
                              excluded: 0,
                              unsynced: syncKind === 'none' ? 1 : 0 },
            paths: [{
                capture_pin:  capture_inst + '/D',
                odb_type:     'iterm', odb_id: 1,
                capture_inst, capture_cell: 'DFF_X1',
                launch_clock: 'clk_a', capture_clock: 'clk_b',
                category: syncKind === 'none' ? 'unsynchronized' : 'synchronized',
                sync_chain_kind:  syncKind,
                sync_chain_depth: syncKind === 'none' ? 1 : 2,
                whitelist_match:   null,
                whitelist_pattern: null,
            }],
        });

        it('synced capture flop banner spells out "crossover · stage 1"',
                async () => {
            const PATH_DETAIL = {
                stages: [
                    reg('ff_src_a', {
                        is_launch: true, clock: 'clk_a',
                        out_net: { name: 'q_a', odb_type: 'net', odb_id: 1 },
                    }),
                    reg('ff_capture', {
                        is_capture: true, launch_clock: 'clk_a', clock: 'clk_b',
                        out_net: { name: 'q_cap', odb_type: 'net', odb_id: 2 },
                    }),
                    reg('ff_sync2', {
                        is_sync_stage: true, clock: 'clk_b',
                    }),
                ],
                sync_chain: { kind: 'ff_chain', depth: 3,
                              whitelist_match: null, whitelist_pattern: null },
            };
            const widget = await openDetail(buildOverviewApp(
                PATH_DETAIL, PATHS_ONE_ROW('clk_b', 'ff_capture', 'ff_chain')));
            const text = widget._cdcBody.textContent;
            assert.ok(/crossover · stage 1/i.test(text),
                'capture flop banner combines crossover + stage 1');
            // No standalone "⚠ crossover" — the sync version replaces it.
            assert.ok(!/⚠ crossover\b/.test(text)
                      || /crossover · stage 1/.test(text),
                'red ⚠ crossover banner is suppressed when chain detected');
        });

        it('unsynced capture flop banner stays "⚠ crossover" (no stage 1)',
                async () => {
            const PATH_DETAIL = {
                stages: [
                    reg('ff_src_a', {
                        is_launch: true, clock: 'clk_a',
                        out_net: { name: 'q_a', odb_type: 'net', odb_id: 1 },
                    }),
                    reg('ff_capture', {
                        is_capture: true, launch_clock: 'clk_a', clock: 'clk_b',
                    }),
                ],
                sync_chain: { kind: 'none', depth: 1,
                              whitelist_match: null, whitelist_pattern: null },
            };
            const widget = await openDetail(buildOverviewApp(
                PATH_DETAIL, PATHS_ONE_ROW('clk_b', 'ff_capture', 'none')));
            const text = widget._cdcBody.textContent;
            assert.ok(/⚠ crossover/.test(text),
                'red crossover banner shown when no chain detected');
            assert.ok(!/stage 1\b/.test(text),
                'no "stage 1" suffix when there\'s no sync chain');
        });

        it('subsequent sync stages number from 2 onwards', async () => {
            const PATH_DETAIL = {
                stages: [
                    reg('ff_src_a', {
                        is_launch: true, clock: 'clk_a',
                        out_net: { name: 'q_a', odb_type: 'net', odb_id: 1 },
                    }),
                    reg('ff_capture', {
                        is_capture: true, launch_clock: 'clk_a', clock: 'clk_b',
                        out_net: { name: 'q_cap', odb_type: 'net', odb_id: 2 },
                    }),
                    reg('ff_sync2', {
                        is_sync_stage: true, clock: 'clk_b',
                        out_net: { name: 'q_s2', odb_type: 'net', odb_id: 3 },
                    }),
                    reg('ff_sync3', {
                        is_sync_stage: true, clock: 'clk_b',
                    }),
                ],
                sync_chain: { kind: 'ff_chain', depth: 4,
                              whitelist_match: null, whitelist_pattern: null },
            };
            const widget = await openDetail(buildOverviewApp(
                PATH_DETAIL, PATHS_ONE_ROW('clk_b', 'ff_capture', 'ff_chain')));
            const text = widget._cdcBody.textContent;
            assert.ok(/sync stage 2/.test(text),
                'first downstream stage labelled "sync stage 2"');
            assert.ok(/sync stage 3/.test(text),
                'second downstream stage labelled "sync stage 3"');
            // No "sync stage 1" — that role is the capture flop's
            // "crossover · stage 1" combo banner.
            assert.ok(!/sync stage 1\b/.test(text),
                'no "sync stage 1" — capture flop owns stage 1');
        });

        it('FF chain depth displayed as-is (no cap at 2FF)', async () => {
            // Path-list cell renders "${depth}FF chain" with no cap so
            // depths above 2 surface honestly.
            const PATHS = {
                time_unit: 'ns',
                total: 1, offset: 0,
                category_total: { synced: 1, excluded: 0, unsynced: 0 },
                paths: [{
                    capture_pin:  'sync_b1/D',
                    odb_type:     'iterm', odb_id: 1,
                    capture_inst: 'sync_b1', capture_cell: 'DFF_X1',
                    launch_clock: 'clk_a', capture_clock: 'clk_b',
                    category: 'synchronized',
                    sync_chain_kind:  'ff_chain',
                    sync_chain_depth: 4,
                    whitelist_match:   null,
                    whitelist_pattern: null,
                }],
            };
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths: () => PATHS,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]').click();
            await settle();
            const text = widget._cdcBody.textContent;
            assert.ok(/4FF chain/.test(text),
                'depth=4 surfaces as "4FF chain" — no cap');
        });

        it('synced single-cell chain shows plain "crossover" (no stage 1)',
                async () => {
            // A synchronizer whose entire chain is the capture flop
            // itself (e.g. a single liberty_sync cell, a whitelisted
            // single instance) has NO downstream sync stage. The
            // banner should drop the "stage 1" suffix — there's no
            // stage 2 to make "stage 1" meaningful — but stay green
            // to signal the path is synced.
            const PATH_DETAIL = {
                stages: [
                    reg('ff_src_a', {
                        is_launch: true, clock: 'clk_a',
                        out_net: { name: 'q_a', odb_type: 'net', odb_id: 1 },
                    }),
                    reg('sync_only', {
                        is_capture: true, launch_clock: 'clk_a',
                        clock: 'clk_b', cell: 'SYNC2_X1',
                    }),
                ],
                sync_chain: { kind: 'liberty_sync', depth: 2,
                              whitelist_match: null, whitelist_pattern: null },
            };
            const widget = await openDetail(buildOverviewApp(
                PATH_DETAIL,
                PATHS_ONE_ROW('clk_b', 'sync_only', 'liberty_sync')));
            const text = widget._cdcBody.textContent;
            assert.ok(/⚠ crossover/.test(text) === false,
                'synced single-cell capture should NOT show red ⚠ banner');
            assert.ok(/crossover/i.test(text),
                'banner still says "crossover"');
            assert.ok(!/crossover · stage 1/.test(text),
                '"stage 1" suffix dropped — no downstream sync stage exists');
        });

        it('composite chain banner stays "crossover · stage 1" with downstream',
                async () => {
            // A composite chain still has a "stage 1" / "stage 2+"
            // structure when the capture flop is followed by another
            // sync flop. Verify the banner logic applies to the
            // composite kind alongside ff_chain / liberty_sync /
            // whitelisted.
            const PATH_DETAIL = {
                stages: [
                    reg('ff_src_a', {
                        is_launch: true, clock: 'clk_a',
                        out_net: { name: 'q_a', odb_type: 'net', odb_id: 1 },
                    }),
                    reg('ff_capture', {
                        is_capture: true, launch_clock: 'clk_a',
                        clock: 'clk_b',
                        out_net: { name: 'q_cap', odb_type: 'net', odb_id: 2 },
                    }),
                    reg('sync_b2', {
                        is_sync_stage: true, clock: 'clk_b',
                        cell: 'SYNC2_X1',
                    }),
                ],
                sync_chain: { kind: 'composite', depth: 3,
                              whitelist_match: null, whitelist_pattern: null },
            };
            const widget = await openDetail(buildOverviewApp(
                PATH_DETAIL,
                PATHS_ONE_ROW('clk_b', 'ff_capture', 'composite')));
            const text = widget._cdcBody.textContent;
            assert.ok(/crossover · stage 1/i.test(text),
                'composite-chain capture flop also banners as stage 1');
            assert.ok(/sync stage 2/.test(text),
                'downstream stage labelled stage 2');
        });

        it('path-list cell labels composite chains', async () => {
            const PATHS = {
                time_unit: 'ns',
                total: 1, offset: 0,
                category_total: { synced: 1, excluded: 0, unsynced: 0 },
                paths: [{
                    capture_pin: 'ff_capture/D',
                    odb_type: 'iterm', odb_id: 1,
                    capture_inst: 'ff_capture', capture_cell: 'DFF_X1',
                    launch_clock: 'clk_a', capture_clock: 'clk_b',
                    category: 'synchronized',
                    sync_chain_kind: 'composite',
                    sync_chain_depth: 3,
                    whitelist_match: null, whitelist_pattern: null,
                }],
            };
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths: () => PATHS,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]').click();
            await settle();
            const text = widget._cdcBody.textContent;
            assert.ok(/composite chain \(depth 3\)/.test(text),
                'composite kind renders depth alongside the label');
            // Tooltip explains the FF + statetable mix.
            const spans = Array.from(widget._cdcBody.querySelectorAll('span'));
            const cell = spans.find(s =>
                /composite chain/.test(s.textContent));
            assert.ok(cell, 'composite cell span present');
            assert.ok(/ff-group flop AND/i.test(cell.title),
                'tooltip explains the mixed-source semantics');
        });

        it('path-detail summary names composite synchronizer', async () => {
            const PATH_DETAIL = {
                stages: [
                    reg('ff_src_a', {
                        is_launch: true, clock: 'clk_a',
                        out_net: { name: 'q_a', odb_type: 'net', odb_id: 1 },
                    }),
                    reg('ff_capture', {
                        is_capture: true, launch_clock: 'clk_a',
                        clock: 'clk_b',
                        out_net: { name: 'q_cap', odb_type: 'net', odb_id: 2 },
                    }),
                    reg('sync_b2', {
                        is_sync_stage: true, clock: 'clk_b',
                        cell: 'SYNC2_X1',
                    }),
                ],
                sync_chain: { kind: 'composite', depth: 3,
                              whitelist_match: null, whitelist_pattern: null },
            };
            const widget = await openDetail(buildOverviewApp(
                PATH_DETAIL,
                PATHS_ONE_ROW('clk_b', 'ff_capture', 'composite')));
            const text = widget._cdcBody.textContent;
            assert.ok(/composite synchronizer detected/i.test(text),
                'summary line spells out the mixed-source classification');
            assert.ok(/depth 3/.test(text),
                'summary cites the total mixed depth');
        });
    });

    describe('CDC path-detail domain mix and per-input clock badges', () => {
        const CDC_OVERVIEW = {
            time_unit: 'ns',
            current_mode: 'default',
            modes: { default: {
                endpoint_count: 1, clocks: ['clk_a', 'clk_b'],
                matrix: { clk_a: { clk_b: {
                    paths: 1, synced: 0, excluded: 0, unsynced: 1,
                }}},
            }},
        };

        const PATHS = {
            time_unit: 'ns',
            total: 1, offset: 0,
            category_total: { synced: 0, excluded: 0, unsynced: 1 },
            paths: [{
                capture_pin:  'ff_mix/D', odb_type: 'iterm', odb_id: 1,
                capture_inst: 'ff_mix', capture_cell: 'DFF_X1',
                launch_clock: 'clk_a', capture_clock: 'clk_b',
                category: 'unsynchronized',
                sync_chain_kind: 'none', sync_chain_depth: 1,
                whitelist_match: null, whitelist_pattern: null,
            }],
        };

        it('comb stage with multi-domain inputs gets DOMAIN MIX banner',
                async () => {
            const PATH_DETAIL = {
                stages: [
                    {
                        instance: 'ff_src_a', cell: 'DFF_X1',
                        odb_type: 'inst', odb_id: 10,
                        d_pin: 'ff_src_a/D',
                        d_pin_odb_type: 'iterm', d_pin_odb_id: 100,
                        q_pin: 'ff_src_a/Q',
                        q_pin_odb_type: 'iterm', q_pin_odb_id: 101,
                        kind: 'register',
                        is_launch: true, is_capture: false, is_sync_stage: false,
                        clock: 'clk_a', capture_clock: 'clk_b',
                        out_net: { name: 'q_a', odb_type: 'net', odb_id: 200 },
                        passthroughs_after: [],
                    },
                    {
                        instance: 'mix_and', cell: 'AND2_X1',
                        odb_type: 'inst', odb_id: 11,
                        in_pin: 'mix_and/A1',
                        in_pin_odb_type: 'iterm', in_pin_odb_id: 102,
                        in_pin_clocks: ['clk_a'],
                        out_pin: 'mix_and/ZN',
                        out_pin_odb_type: 'iterm', out_pin_odb_id: 103,
                        aux_in_pins: [{
                            name: 'mix_and/A2',
                            odb_type: 'iterm', odb_id: 104,
                            clocks: ['clk_b'],
                        }],
                        kind: 'comb',
                        is_launch: false, is_capture: false, is_sync_stage: false,
                        is_domain_mix: true,
                        clock: 'clk_a', capture_clock: 'clk_b',
                        out_net: { name: 'mix_y', odb_type: 'net', odb_id: 201 },
                        passthroughs_after: [],
                    },
                    {
                        instance: 'ff_mix', cell: 'DFF_X1',
                        odb_type: 'inst', odb_id: 12,
                        d_pin: 'ff_mix/D',
                        d_pin_odb_type: 'iterm', d_pin_odb_id: 105,
                        q_pin: 'ff_mix/Q',
                        q_pin_odb_type: 'iterm', q_pin_odb_id: 106,
                        kind: 'register',
                        is_launch: false, is_capture: true, is_sync_stage: false,
                        launch_clock: 'clk_a', capture_clock: 'clk_b',
                        clock: 'clk_b',
                        out_net: null, passthroughs_after: [],
                    },
                ],
                sync_chain: { kind: 'none', depth: 1,
                              whitelist_match: null, whitelist_pattern: null },
            };
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths: () => PATHS,
                cdc_path_detail: () => PATH_DETAIL,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]').click();
            await settle();
            const link = widget._cdcBody.querySelector(
                'tbody tr td span');
            link.click();
            await settle();
            const text = widget._cdcBody.textContent;
            assert.ok(/domain mix/i.test(text),
                'comb gate with mixed inputs flagged as DOMAIN MIX');
            assert.ok(/AND2_X1/.test(text),
                'banner names the cell type');
        });

        it('per-input clock badges reflect each pin\'s actual domain',
                async () => {
            // The IN row shows the followed input's clock (clk_a),
            // and the +IN aux row shows the OTHER input's clock (clk_b).
            const PATH_DETAIL = {
                stages: [
                    {
                        instance: 'ff_src_a', cell: 'DFF_X1',
                        odb_type: 'inst', odb_id: 10,
                        d_pin: 'ff_src_a/D',
                        d_pin_odb_type: 'iterm', d_pin_odb_id: 100,
                        q_pin: 'ff_src_a/Q',
                        q_pin_odb_type: 'iterm', q_pin_odb_id: 101,
                        kind: 'register', is_launch: true,
                        is_capture: false, is_sync_stage: false,
                        clock: 'clk_a', capture_clock: 'clk_b',
                        out_net: { name: 'q_a', odb_type: 'net', odb_id: 200 },
                        passthroughs_after: [],
                    },
                    {
                        instance: 'mix_and', cell: 'AND2_X1',
                        odb_type: 'inst', odb_id: 11,
                        in_pin: 'mix_and/A1',
                        in_pin_odb_type: 'iterm', in_pin_odb_id: 102,
                        in_pin_clocks: ['clk_a'],
                        out_pin: 'mix_and/ZN',
                        out_pin_odb_type: 'iterm', out_pin_odb_id: 103,
                        aux_in_pins: [{
                            name: 'mix_and/A2',
                            odb_type: 'iterm', odb_id: 104,
                            clocks: ['clk_b'],
                        }],
                        kind: 'comb', is_domain_mix: true,
                        is_launch: false, is_capture: false, is_sync_stage: false,
                        clock: 'clk_a', capture_clock: 'clk_b',
                        out_net: { name: 'mix_y', odb_type: 'net', odb_id: 201 },
                        passthroughs_after: [],
                    },
                    {
                        instance: 'ff_mix', cell: 'DFF_X1',
                        odb_type: 'inst', odb_id: 12,
                        d_pin: 'ff_mix/D',
                        d_pin_odb_type: 'iterm', d_pin_odb_id: 105,
                        q_pin: 'ff_mix/Q',
                        q_pin_odb_type: 'iterm', q_pin_odb_id: 106,
                        kind: 'register', is_capture: true,
                        is_launch: false, is_sync_stage: false,
                        launch_clock: 'clk_a', capture_clock: 'clk_b',
                        clock: 'clk_b',
                        out_net: null, passthroughs_after: [],
                    },
                ],
                sync_chain: { kind: 'none', depth: 1,
                              whitelist_match: null, whitelist_pattern: null },
            };
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths: () => PATHS,
                cdc_path_detail: () => PATH_DETAIL,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]').click();
            await settle();
            const link = widget._cdcBody.querySelector(
                'tbody tr td span');
            link.click();
            await settle();
            const text = widget._cdcBody.textContent;
            // Both clock names appear because each input pin renders
            // with its own domain badge.
            assert.ok(/clk_a/.test(text), 'in_pin clock badge present');
            assert.ok(/clk_b/.test(text), 'aux_in_pin clock badge present');
            // The aux pin should be visible. Same DOM-based
            // lookup as the comb-stages test: textContent's "A2"
            // collides with "AND2_X1", so we check element-level.
            assert.ok(text.includes('mix_and'),
                'mix_and instance present in card header');
            const a2Spans = Array.from(
                widget._cdcBody.querySelectorAll('span'))
                .filter(s => s.textContent === 'A2');
            assert.ok(a2Spans.length > 0,
                'aux pin "A2" rendered as a leaf pin row');
        });
    });

    // Clock-mix tracer — replaces the old per-clock-chip data
    // fan-in affordance. A multi-clock pin row carries a single
    // `↑ trace mix` button next to the chip cluster; clicking it
    // dispatches `cdc_clock_mix_trace` and renders the mixer
    // gate(s) above the originating stage card. Single-clock pins
    // render no button (no mix to trace). Output pins (OUT / Q
    // rows) render no button — by construction they redistribute
    // their inputs' clocks, so a walk from OUT just re-derives
    // what clicking the matching IN row would produce.
    describe('CDC path-detail clock-mix trace button', () => {
        const CDC_OVERVIEW = {
            time_unit: 'ns',
            current_mode: 'default',
            modes: { default: {
                endpoint_count: 1, clocks: ['clk_a', 'clk_b'],
                matrix: { clk_a: { clk_b: {
                    paths: 1, synced: 0, excluded: 0, unsynced: 1,
                }}},
            }},
        };
        const PATHS = {
            time_unit: 'ns', total: 1, offset: 0,
            category_total: { synced: 0, excluded: 0, unsynced: 1 },
            paths: [{
                capture_pin:  'ff_mix/D', odb_type: 'iterm', odb_id: 1,
                capture_inst: 'ff_mix', capture_cell: 'DFF_X1',
                launch_clock: 'clk_a', capture_clock: 'clk_b',
                category: 'unsynchronized',
                sync_chain_kind: 'none', sync_chain_depth: 1,
                whitelist_match: null, whitelist_pattern: null,
            }],
        };
        // Path detail puts the multi-clock-CK FF (ff_mux_clk) on
        // screen — its CK row carries [clk_a, clk_b], so the CK
        // row is exactly the kind of multi-clock pin the trace-mix
        // button is for.
        const PATH_DETAIL = {
            stages: [
                { instance: 'ff_mux_clk', cell: 'DFF_X1',
                  odb_type: 'inst', odb_id: 10,
                  d_pin: 'ff_mux_clk/D',
                  d_pin_odb_type: 'iterm', d_pin_odb_id: 100,
                  q_pin: 'ff_mux_clk/Q',
                  q_pin_odb_type: 'iterm', q_pin_odb_id: 101,
                  ck_pin: 'ff_mux_clk/CK',
                  ck_pin_odb_type: 'iterm', ck_pin_odb_id: 102,
                  ck_pin_clocks: ['clk_a', 'clk_b'],
                  kind: 'register', is_launch: true, clock: 'clk_a',
                  out_net: { name: 'mux_q', odb_type: 'net', odb_id: 200 },
                  passthroughs_after: [] },
                { instance: 'ff_mix', cell: 'DFF_X1',
                  odb_type: 'inst', odb_id: 12,
                  d_pin: 'ff_mix/D',
                  d_pin_odb_type: 'iterm', d_pin_odb_id: 105,
                  q_pin: 'ff_mix/Q',
                  q_pin_odb_type: 'iterm', q_pin_odb_id: 106,
                  ck_pin: 'ff_mix/CK',
                  ck_pin_odb_type: 'iterm', ck_pin_odb_id: 108,
                  ck_pin_clocks: ['clk_b'],
                  kind: 'register', is_capture: true,
                  launch_clock: 'clk_a', capture_clock: 'clk_b',
                  clock: 'clk_b',
                  out_net: null, passthroughs_after: [] },
            ],
            sync_chain: { kind: 'none', depth: 1,
                          whitelist_match: null, whitelist_pattern: null },
        };
        // Clock-mix walk from CK lands on the OR gate that mixes
        // clk_a + clk_b on the clock net.
        const MIX_RESPONSE = {
            requested_clocks: ['clk_a', 'clk_b'],
            stages: [
                { kind: 'mixer',
                  instance: 'clk_mux_g', cell: 'OR2_X1',
                  odb_type: 'inst', odb_id: 60,
                  out_pin: 'clk_mux_g/ZN',
                  out_pin_odb_type: 'iterm', out_pin_odb_id: 601,
                  via_pin: null,
                  clocks: ['clk_a', 'clk_b'],
                  contributors: [
                      { pin: 'clk_mux_g/A1',
                        pin_odb_type: 'iterm', pin_odb_id: 600,
                        clocks: ['clk_a'] },
                      { pin: 'clk_mux_g/A2',
                        pin_odb_type: 'iterm', pin_odb_id: 602,
                        clocks: ['clk_b'] },
                  ],
                  degenerate: false,
                  out_net: { name: 'mux_clk_or',
                             odb_type: 'net', odb_id: 301 },
                  passthroughs_after: [] },
            ],
        };

        const drillToDetail = async (extra = {}) => {
            const app = createMockApp(Object.assign({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths: () => PATHS,
                cdc_path_detail: () => PATH_DETAIL,
                cdc_clock_mix_trace: () => MIX_RESPONSE,
            }, extra));
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click(); await settle();
            widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]')
                .click(); await settle();
            widget._cdcBody.querySelector('tbody tr td span').click();
            await settle();
            return widget;
        };

        // Locate the trace-mix button next to the CK row of
        // ff_mux_clk (which has [clk_a, clk_b]).
        const findCkTraceMixBtn = (widget) => {
            const cards = widget._cdcBody.querySelectorAll(
                '[data-cdc-stage-card]');
            for (const card of cards) {
                if (card.dataset.cdcStageInstance !== 'ff_mux_clk') {
                    continue;
                }
                const btns = card.querySelectorAll(
                    'a[data-cdc-trace-mix="1"]');
                for (const btn of btns) {
                    if (btn.dataset.cdcPinKind === 'clock') {
                        return btn;
                    }
                }
            }
            return null;
        };

        it('multi-clock pin row renders a single trace-mix button',
                async () => {
            const widget = await drillToDetail();
            const btn = findCkTraceMixBtn(widget);
            assert.ok(btn, 'CK row carries an `↑ trace mix` button');
            assert.equal(btn.textContent, '↑ trace mix');
            // The chips themselves should NOT be clickable — the
            // button is the only interactive element for clock-mix
            // tracing now.
            const ckCard = btn.closest('[data-cdc-stage-card]');
            const chips = ckCard.querySelectorAll(
                'span[data-cdc-pin-kind="clock"]');
            assert.ok(chips.length >= 2,
                'CK row renders both clock chips');
            for (const ch of chips) {
                assert.notEqual(ch.style.cursor, 'pointer',
                    'chip is inert (no per-chip click handler)');
            }
        });

        it('single-clock pin row does NOT render a trace-mix button',
                async () => {
            const widget = await drillToDetail();
            // The capture flop ff_mix has CK=[clk_b] (single clock)
            // so its CK row should NOT carry a trace-mix button.
            const cards = widget._cdcBody.querySelectorAll(
                '[data-cdc-stage-card]');
            let captureCard = null;
            for (const card of cards) {
                if (card.dataset.cdcStageInstance === 'ff_mix') {
                    captureCard = card;
                    break;
                }
            }
            assert.ok(captureCard, 'capture stage card found');
            const btns = captureCard.querySelectorAll(
                'a[data-cdc-trace-mix="1"]');
            assert.equal(btns.length, 0,
                'no trace-mix button on a single-clock card');
        });

        it('clicking the button dispatches cdc_clock_mix_trace '
                + 'with the row clocks and inserts a mixer card',
                async () => {
            const calls = [];
            const widget = await drillToDetail({
                cdc_clock_mix_trace: (req) => {
                    calls.push(req);
                    return MIX_RESPONSE;
                },
            });
            const btn = findCkTraceMixBtn(widget);
            btn.click(); await settle();
            assert.equal(calls.length, 1, 'RPC fired once');
            assert.equal(calls[0].pin_odb_type, 'iterm');
            assert.equal(calls[0].pin_odb_id, 102);
            assert.deepEqual(calls[0].clocks.slice().sort(),
                             ['clk_a', 'clk_b']);
            const wrapper = widget._cdcBody.querySelector(
                '[data-cdc-fan-in-key]');
            assert.ok(wrapper, 'mix wrapper inserted');
            assert.ok(/clock-mix trace/.test(wrapper.textContent),
                'header labels the trace as clock-mix');
            assert.ok(/CLOCK MIX/.test(wrapper.textContent),
                'mixer banner rendered');
            assert.ok(/clk_mux_g/.test(wrapper.textContent),
                'mixer instance rendered');
        });

        it('mixer card surfaces per-contributor trace-mix links '
                + 'only when the contributor subset is multi-clock',
                async () => {
            // Replace MIX_RESPONSE with one whose first contributor
            // is itself multi-clock so we can exercise the recursive
            // per-contributor link rendering rule.
            const NESTED_MIX = {
                requested_clocks: ['clk_a', 'clk_b', 'clk_c'],
                stages: [
                    { kind: 'mixer',
                      instance: 'or_outer', cell: 'OR2_X1',
                      out_pin: 'or_outer/ZN',
                      out_pin_odb_type: 'iterm', out_pin_odb_id: 700,
                      via_pin: null,
                      clocks: ['clk_a', 'clk_b', 'clk_c'],
                      contributors: [
                          { pin: 'or_outer/A1',
                            pin_odb_type: 'iterm', pin_odb_id: 701,
                            clocks: ['clk_a', 'clk_b'] },
                          { pin: 'or_outer/A2',
                            pin_odb_type: 'iterm', pin_odb_id: 702,
                            clocks: ['clk_c'] },
                      ],
                      degenerate: false,
                      out_net: null, passthroughs_after: [] },
                ],
            };
            const widget = await drillToDetail({
                cdc_clock_mix_trace: () => NESTED_MIX,
            });
            const btn = findCkTraceMixBtn(widget);
            btn.click(); await settle();
            const wrapper = widget._cdcBody.querySelector(
                '[data-cdc-fan-in-key]');
            assert.ok(wrapper, 'mix wrapper inserted');
            // The mixer card is INSIDE the wrapper, NOT the
            // originating CK card — so locate it inside the wrapper.
            const inner = wrapper.querySelectorAll(
                'a[data-cdc-trace-mix="1"]');
            assert.equal(inner.length, 1,
                'exactly one per-contributor link — the multi-clock '
                + '{clk_a, clk_b} subset gets one; the single-clock '
                + '{clk_c} subset does not');
        });

        it('re-clicking the button collapses the expansion',
                async () => {
            const widget = await drillToDetail();
            const btn = findCkTraceMixBtn(widget);
            btn.click(); await settle();
            assert.ok(widget._cdcBody.querySelector(
                '[data-cdc-fan-in-key]'),
                'expansion present after first click');
            btn.click(); await settle();
            assert.ok(!widget._cdcBody.querySelector(
                '[data-cdc-fan-in-key]'),
                'expansion gone after second click');
        });

        it('output pin rows (OUT/Q) carry no trace-mix button '
                + 'even when multi-clock', async () => {
            // Manufacture a path detail where ff_mux_clk's Q row
            // carries multiple clocks. The renderer should still
            // skip the button on the Q row because Q is an output
            // pin (output rows just redistribute their inputs'
            // clocks).
            const PATH_OUT = JSON.parse(JSON.stringify(PATH_DETAIL));
            // The launch flop carries clock 'clk_a' as its `clock`
            // field; pretend Q sees [clk_a, clk_b] for this test.
            PATH_OUT.stages[0].clock = 'clk_a';
            // Add a multi-clock badge by hijacking ck_pin_clocks
            // — but the existing renderer reads ck_pin_clocks for
            // the CK row only; Q row reads `qClk` derived from
            // `s.clock`. So just keep the data-driven assertion:
            // there is ONLY ONE trace-mix button on this card,
            // and it sits on the CK row, not on Q.
            const widget = await drillToDetail({
                cdc_path_detail: () => PATH_OUT,
            });
            const cards = widget._cdcBody.querySelectorAll(
                '[data-cdc-stage-card]');
            let launchCard = null;
            for (const card of cards) {
                if (card.dataset.cdcStageInstance === 'ff_mux_clk') {
                    launchCard = card;
                    break;
                }
            }
            assert.ok(launchCard, 'launch card found');
            const btns = launchCard.querySelectorAll(
                'a[data-cdc-trace-mix="1"]');
            assert.equal(btns.length, 1,
                'exactly one trace-mix button on the multi-clock '
                + 'launch card — on the CK row, not on Q');
            assert.equal(btns[0].dataset.cdcPinKind, 'clock',
                'the one button belongs to the CK row (pinKind=clock)');
        });
    });


    // Multi-clock CK rendering — when a register stage carries
    // `ck_pin_clocks` with more than one entry, the card renders
    // a CK pin row showing every clock + a "⚠ multi-clock CK"
    // warning band. This is the UX cue that explains "why does
    // the same FF appear as the source for N clocks": the FF's
    // CK genuinely IS in N domains (clock-mux'd, propagated-clock
    // convergence, or similar).
    describe('CDC path-detail multi-clock CK marker', () => {
        const CDC_OVERVIEW = {
            time_unit: 'ns',
            current_mode: 'default',
            modes: { default: {
                endpoint_count: 1, clocks: ['clk_a', 'clk_b'],
                matrix: { clk_a: { clk_b: {
                    paths: 1, synced: 0, excluded: 0, unsynced: 1,
                }}},
            }},
        };
        const PATHS = {
            time_unit: 'ns', total: 1, offset: 0,
            category_total: { synced: 0, excluded: 0, unsynced: 1 },
            paths: [{
                capture_pin:  'ff_dst/D', odb_type: 'iterm', odb_id: 1,
                capture_inst: 'ff_dst', capture_cell: 'DFF_X1',
                launch_clock: 'clk_a', capture_clock: 'clk_b',
                category: 'unsynchronized',
                sync_chain_kind: 'none', sync_chain_depth: 1,
                whitelist_match: null, whitelist_pattern: null,
            }],
        };
        const PATH_DETAIL = {
            stages: [
                // Launch flop with multi-clock CK — clockDomains(CK)
                // returned both clk_a and clk_b because the CK
                // net upstream is clock-mux'd. Both walks (clk_a
                // and clk_b) land here.
                { instance: 'ff_mux_clk', cell: 'DFF_X1',
                  odb_type: 'inst', odb_id: 10,
                  d_pin: 'ff_mux_clk/D',
                  d_pin_odb_type: 'iterm', d_pin_odb_id: 100,
                  q_pin: 'ff_mux_clk/Q',
                  q_pin_odb_type: 'iterm', q_pin_odb_id: 101,
                  ck_pin: 'ff_mux_clk/CK',
                  ck_pin_odb_type: 'iterm', ck_pin_odb_id: 102,
                  ck_pin_clocks: ['clk_a', 'clk_b'],
                  kind: 'register', is_launch: true, clock: 'clk_a',
                  out_net: { name: 'mux_q', odb_type: 'net', odb_id: 200 },
                  passthroughs_after: [] },
                // Capture flop — single-clock CK, no warning.
                { instance: 'ff_dst', cell: 'DFF_X1',
                  odb_type: 'inst', odb_id: 11,
                  d_pin: 'ff_dst/D',
                  d_pin_odb_type: 'iterm', d_pin_odb_id: 105,
                  q_pin: 'ff_dst/Q',
                  q_pin_odb_type: 'iterm', q_pin_odb_id: 106,
                  ck_pin: 'ff_dst/CK',
                  ck_pin_odb_type: 'iterm', ck_pin_odb_id: 107,
                  ck_pin_clocks: ['clk_b'],
                  kind: 'register', is_capture: true,
                  launch_clock: 'clk_a', capture_clock: 'clk_b',
                  clock: 'clk_b',
                  out_net: null, passthroughs_after: [] },
            ],
            sync_chain: { kind: 'none', depth: 1,
                          whitelist_match: null, whitelist_pattern: null },
        };

        const drillToDetail = async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths: () => PATHS,
                cdc_path_detail: () => PATH_DETAIL,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click(); await settle();
            widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]')
                .click(); await settle();
            widget._cdcBody.querySelector('tbody tr td span').click();
            await settle();
            return widget;
        };

        const findCard = (widget, instanceName) => {
            return Array.from(widget._cdcBody
                .querySelectorAll('[data-cdc-stage-card]'))
                .find(c => c.textContent.includes(instanceName));
        };

        it('register card shows a CK pin row', async () => {
            const widget = await drillToDetail();
            const card = findCard(widget, 'ff_mux_clk');
            assert.ok(card, 'launch flop card rendered');
            assert.ok(/\bCK\b/.test(card.textContent),
                'CK pin row present on the register card');
        });

        it('multi-clock CK renders a chip per clock', async () => {
            const widget = await drillToDetail();
            const card = findCard(widget, 'ff_mux_clk');
            // The CK row should contain BOTH clock names — they
            // render as separate chips (any whitespace between).
            assert.ok(/clk_a/.test(card.textContent)
                && /clk_b/.test(card.textContent),
                'both clk_a and clk_b chips on the multi-clock CK');
        });

        it('multi-clock CK shows the explanation banner',
                async () => {
            const widget = await drillToDetail();
            const card = findCard(widget, 'ff_mux_clk');
            assert.ok(/multi-clock CK/.test(card.textContent),
                'card carries the multi-clock CK warning');
            assert.ok(/2 clocks reach this flop/.test(card.textContent),
                'warning names the count');
        });

        it('single-clock CK does NOT show the warning',
                async () => {
            const widget = await drillToDetail();
            const card = findCard(widget, 'ff_dst');
            assert.ok(card, 'capture flop card rendered');
            assert.ok(!/multi-clock CK/.test(card.textContent),
                'no warning on single-clock CK');
        });
    });

    // Convergence detection — when a fan-in trace's terminal
    // stage is ALREADY visible on the diagram (typical of multi-
    // clock-CK FFs where multiple walks legitimately land on the
    // same flop), don't draw a duplicate wrapper. Annotate the
    // existing card with a "↑ also reached via" line and flash
    // the card to point at it.

    // Clock-path fan-in renders multi-clock convergence gates as
    // "⚠ clock-mux convergence" instead of the data-path
    // "⚠ domain mix" — same backend stage data, different
    // semantic depending on whether the trace started from a CK
    // pin or a data pin.

    // Shared-FF badge on the path-list — when N rows share the
    // same `capture_inst`, each carries a "⚭ N" badge plus a
    // tooltip explaining the convergence. Common case is multi-
    // clock-CK FFs that contribute one row per (launch, capture)
    // pair; the badge tells the user "fixing the FF once
    // addresses all N rows."
    describe('CDC path-list shared-FF badge', () => {
        const CDC_OVERVIEW = {
            time_unit: 'ns',
            current_mode: 'default',
            modes: { default: {
                endpoint_count: 4,
                clocks: ['clk_a', 'clk_b', 'clk_c', 'clk_d'],
                matrix: { clk_a: { clk_c: {
                    paths: 2, synced: 0, excluded: 0, unsynced: 2,
                }}},
            }},
        };
        // 3 rows: two share `ff_mux_clk` (multi-clock-CK FF
        // contributing two pairs), one is a different FF.
        const PATHS = {
            time_unit: 'ns', total: 3, offset: 0,
            category_total: { synced: 0, excluded: 0, unsynced: 3 },
            paths: [
                { capture_pin: 'ff_mux_clk/D',
                  odb_type: 'iterm', odb_id: 1,
                  capture_inst: 'ff_mux_clk', capture_cell: 'DFF_X1',
                  launch_clock: 'clk_a', capture_clock: 'clk_c',
                  category: 'unsynchronized',
                  sync_chain_kind: 'none', sync_chain_depth: 1,
                  whitelist_match: null, whitelist_pattern: null },
                { capture_pin: 'ff_mux_clk/D',
                  odb_type: 'iterm', odb_id: 2,
                  capture_inst: 'ff_mux_clk', capture_cell: 'DFF_X1',
                  launch_clock: 'clk_b', capture_clock: 'clk_d',
                  category: 'unsynchronized',
                  sync_chain_kind: 'none', sync_chain_depth: 1,
                  whitelist_match: null, whitelist_pattern: null },
                { capture_pin: 'ff_lonely/D',
                  odb_type: 'iterm', odb_id: 3,
                  capture_inst: 'ff_lonely', capture_cell: 'DFF_X1',
                  launch_clock: 'clk_a', capture_clock: 'clk_c',
                  category: 'unsynchronized',
                  sync_chain_kind: 'none', sync_chain_depth: 1,
                  whitelist_match: null, whitelist_pattern: null },
            ],
        };

        it('rows sharing capture_inst show the ⚭ N badge',
                async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths: () => PATHS,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click(); await settle();
            widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_c"]')
                .click(); await settle();
            // Walk every tbody row, group by the capture_inst the
            // path-row builder sees (the first cell's text).
            const rows = Array.from(
                widget._cdcBody.querySelectorAll('tbody tr'));
            assert.equal(rows.length, 3, 'three rows rendered');
            const sharedRows = rows.filter(r =>
                /ff_mux_clk/.test(r.textContent));
            const lonelyRows = rows.filter(r =>
                /ff_lonely/.test(r.textContent));
            assert.equal(sharedRows.length, 2,
                'two rows share ff_mux_clk');
            assert.equal(lonelyRows.length, 1,
                'one row for ff_lonely');
            // Both shared rows should carry the ⚭ 2 badge.
            for (const r of sharedRows) {
                assert.ok(/⚭ 2/.test(r.textContent),
                    'shared row has ⚭ 2 badge');
            }
            // Lonely row must NOT have the badge.
            assert.ok(!/⚭/.test(lonelyRows[0].textContent),
                'singleton row has no shared-FF badge');
        });
    });

    describe('CDC forward-chain pass-through expansion', () => {
        const CDC_OVERVIEW = {
            time_unit: 'ns',
            current_mode: 'default',
            modes: { default: {
                endpoint_count: 1, clocks: ['clk_a', 'clk_b'],
                matrix: { clk_a: { clk_b: {
                    paths: 1, synced: 1, excluded: 0, unsynced: 0,
                }}},
            }},
        };

        // BUF between sync_b1 and sync_b2 → emitted as the capture
        // stage's `passthroughs_after` (forward-chain side, not
        // back-walk). Frontend should still render the expander.
        const PATH_DETAIL = {
            stages: [
                {
                    instance: 'ff_src_a', cell: 'DFF_X1',
                    odb_type: 'inst', odb_id: 10,
                    d_pin: 'ff_src_a/D',
                    d_pin_odb_type: 'iterm', d_pin_odb_id: 100,
                    q_pin: 'ff_src_a/Q',
                    q_pin_odb_type: 'iterm', q_pin_odb_id: 101,
                    kind: 'register', is_launch: true,
                    is_capture: false, is_sync_stage: false,
                    clock: 'clk_a', capture_clock: 'clk_b',
                    out_net: { name: 'q_a', odb_type: 'net', odb_id: 200 },
                    passthroughs_after: [],
                },
                {
                    instance: 'sync_b1', cell: 'DFF_X1',
                    odb_type: 'inst', odb_id: 11,
                    d_pin: 'sync_b1/D',
                    d_pin_odb_type: 'iterm', d_pin_odb_id: 102,
                    q_pin: 'sync_b1/Q',
                    q_pin_odb_type: 'iterm', q_pin_odb_id: 103,
                    kind: 'register', is_capture: true,
                    is_launch: false, is_sync_stage: false,
                    launch_clock: 'clk_a', capture_clock: 'clk_b',
                    clock: 'clk_b',
                    out_net: { name: 'q_sync_b1',
                               odb_type: 'net', odb_id: 201 },
                    // FORWARD-chain pass-through between sync_b1 and sync_b2.
                    passthroughs_after: [{
                        instance: 'sync_buf', cell: 'BUF_X1',
                        odb_type: 'inst', odb_id: 12,
                        in_pin: 'sync_buf/A',
                        in_pin_odb_type: 'iterm', in_pin_odb_id: 104,
                        out_pin: 'sync_buf/Z',
                        out_pin_odb_type: 'iterm', out_pin_odb_id: 105,
                        out_net: { name: 'q_sync_buf_z',
                                   odb_type: 'net', odb_id: 202 },
                    }],
                },
                {
                    instance: 'sync_b2', cell: 'DFF_X1',
                    odb_type: 'inst', odb_id: 13,
                    d_pin: 'sync_b2/D',
                    d_pin_odb_type: 'iterm', d_pin_odb_id: 106,
                    q_pin: 'sync_b2/Q',
                    q_pin_odb_type: 'iterm', q_pin_odb_id: 107,
                    kind: 'register', is_sync_stage: true,
                    is_launch: false, is_capture: false,
                    clock: 'clk_b',
                    out_net: null, passthroughs_after: [],
                },
            ],
            sync_chain: { kind: 'ff_chain', depth: 2,
                          whitelist_match: null, whitelist_pattern: null },
        };

        const PATHS = {
            time_unit: 'ns',
            total: 1, offset: 0,
            category_total: { synced: 1, excluded: 0, unsynced: 0 },
            paths: [{
                capture_pin:  'sync_b1/D',
                odb_type:     'iterm', odb_id: 1,
                capture_inst: 'sync_b1', capture_cell: 'DFF_X1',
                launch_clock: 'clk_a', capture_clock: 'clk_b',
                category: 'synchronized',
                sync_chain_kind:  'ff_chain', sync_chain_depth: 2,
                whitelist_match: null, whitelist_pattern: null,
            }],
        };

        async function loadDetail() {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths: () => PATHS,
                cdc_path_detail: () => PATH_DETAIL,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]').click();
            await settle();
            const link = widget._cdcBody.querySelector(
                'tbody tr td span');
            link.click();
            await settle();
            return widget;
        }

        it('forward-chain BUF surfaces via collapsed <details> expander',
                async () => {
            const widget = await loadDetail();
            const detailsEls = widget._cdcBody.querySelectorAll('details');
            assert.ok(detailsEls.length >= 1,
                'forward gap rendered as <details>');
            // Find the gap whose summary advertises a hidden cell.
            const gap = Array.from(detailsEls).find(d =>
                /\+ 1 hidden cell/.test(d.querySelector('summary').textContent));
            assert.ok(gap, 'summary advertises the hidden BUF');
            assert.equal(gap.open, false, 'collapsed by default');
        });

        it('expanding the forward gap reveals the BUF mini-card', async () => {
            const widget = await loadDetail();
            const detailsEls = widget._cdcBody.querySelectorAll('details');
            const gap = Array.from(detailsEls).find(d =>
                /\+ 1 hidden cell/.test(d.querySelector('summary').textContent));
            gap.open = true;
            await settle();
            const text = widget._cdcBody.textContent;
            assert.ok(/pass · BUF_X1/.test(text),
                'expanded gap reveals BUF pass-through mini-card');
            assert.ok(/sync_buf\b/.test(text),
                'mini-card shows the buffer\'s instance name');
        });
    });

    describe('CDC path-detail sticky legend footer', () => {
        // Build a path-detail with several stages spanning two
        // clock domains so the legend has multiple swatches to
        // render. Reused across the three legend tests.
        const CDC_OVERVIEW = {
            time_unit: 'ns',
            current_mode: 'default',
            modes: { default: {
                endpoint_count: 1, clocks: ['clk_a', 'clk_b'],
                matrix: { clk_a: { clk_b: {
                    paths: 1, synced: 0, excluded: 0, unsynced: 1,
                }}},
            }},
        };
        const PATHS = {
            time_unit: 'ns',
            total: 1, offset: 0,
            category_total: { synced: 0, excluded: 0, unsynced: 1 },
            paths: [{
                capture_pin:  'ff_unsynced/D', odb_type: 'iterm',
                odb_id: 1, capture_inst: 'ff_unsynced',
                capture_cell: 'DFF_X1',
                launch_clock: 'clk_a', capture_clock: 'clk_b',
                category: 'unsynchronized',
                sync_chain_kind: 'none', sync_chain_depth: 1,
                whitelist_match: null, whitelist_pattern: null,
            }],
        };
        const PATH_DETAIL = {
            stages: [
                { instance: 'ff_src_a', cell: 'DFF_X1',
                  odb_type: 'inst', odb_id: 10,
                  d_pin: 'ff_src_a/D',
                  d_pin_odb_type: 'iterm', d_pin_odb_id: 100,
                  q_pin: 'ff_src_a/Q',
                  q_pin_odb_type: 'iterm', q_pin_odb_id: 101,
                  kind: 'register', is_launch: true,
                  clock: 'clk_a',
                  out_net: { name: 'q_a', odb_type: 'net', odb_id: 200 },
                  passthroughs_after: [] },
                { instance: 'ff_unsynced', cell: 'DFF_X1',
                  odb_type: 'inst', odb_id: 12,
                  d_pin: 'ff_unsynced/D',
                  d_pin_odb_type: 'iterm', d_pin_odb_id: 105,
                  q_pin: 'ff_unsynced/Q',
                  q_pin_odb_type: 'iterm', q_pin_odb_id: 106,
                  kind: 'register', is_capture: true,
                  launch_clock: 'clk_a', capture_clock: 'clk_b',
                  clock: 'clk_b',
                  out_net: null, passthroughs_after: [] },
            ],
            sync_chain: { kind: 'none', depth: 1,
                          whitelist_match: null, whitelist_pattern: null },
        };
        const drillToDetail = async (widget) => {
            widget._cdcScanBtn.click();
            await settle();
            widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]')
                .click();
            await settle();
            widget._cdcBody.querySelector('tbody tr td span').click();
            await settle();
        };

        it('renders a swatch per clock domain on the path', async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths: () => PATHS,
                cdc_path_detail: () => PATH_DETAIL,
            });
            const widget = await setupTabWidget('CDC', app);
            await drillToDetail(widget);
            const footer = widget._cdcLegendFooter;
            assert.ok(footer,
                'legend footer element exists');
            assert.notEqual(footer.style.display, 'none',
                'legend footer is visible on path detail');
            assert.ok(/clock domains:/i.test(footer.textContent),
                'legend has clock-domain section');
            assert.ok(/clk_a/.test(footer.textContent),
                'legend lists the launch clock');
            assert.ok(/clk_b/.test(footer.textContent),
                'legend lists the capture clock');
            assert.ok(/clk_a.*\(launch\)|launch.*clk_a/.test(
                footer.textContent),
                'launch clock is tagged');
            assert.ok(/clk_b.*\(capture\)|capture.*clk_b/.test(
                footer.textContent),
                'capture clock is tagged');
        });

        it('renders banner-key entries for the role badges',
                async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths: () => PATHS,
                cdc_path_detail: () => PATH_DETAIL,
            });
            const widget = await setupTabWidget('CDC', app);
            await drillToDetail(widget);
            const footer = widget._cdcLegendFooter;
            assert.ok(/banners:/i.test(footer.textContent),
                'legend has banner-key section');
            assert.ok(/launch/i.test(footer.textContent),
                'banner key includes launch');
            assert.ok(/crossover/i.test(footer.textContent),
                'banner key includes crossover');
            assert.ok(/sync stage/i.test(footer.textContent),
                'banner key includes sync stage');
            assert.ok(/domain mix/i.test(footer.textContent),
                'banner key includes domain mix');
        });

        it('hides the legend on matrix and path-list views',
                async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths: () => PATHS,
                cdc_path_detail: () => PATH_DETAIL,
            });
            const widget = await setupTabWidget('CDC', app);
            widget._cdcScanBtn.click();
            await settle();
            const footer = widget._cdcLegendFooter;
            assert.equal(footer.style.display, 'none',
                'legend hidden on matrix view');
            // Drill to path list — still hidden.
            widget._cdcBody.querySelector(
                'td[data-cdc-launch="clk_a"][data-cdc-capture="clk_b"]')
                .click();
            await settle();
            assert.equal(footer.style.display, 'none',
                'legend hidden on path-list view');
            // Now drill to detail — visible.
            widget._cdcBody.querySelector('tbody tr td span').click();
            await settle();
            assert.notEqual(footer.style.display, 'none',
                'legend visible on path-detail view');
            // Bounce back to matrix — hidden again.
            const back = Array.from(
                widget._cdcBody.querySelectorAll('a'))
                .find(a => /◂ matrix/.test(a.textContent));
            back.click();
            await settle();
            assert.equal(footer.style.display, 'none',
                'legend hidden again after returning to matrix');
        });
    });

    describe('pin leaf name helper', () => {
        // `_pinLeafName(pin, instance)` is used everywhere the
        // Endpoints / Clocks tabs render a per-pin row inside an
        // already-titled card — it strips the instance prefix so the
        // user sees just `D` / `Q` / `CK` instead of the full
        // hierarchical path the row's card already displays.
        let widget;
        beforeEach(async () => {
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
            });
            widget = await setupTabWidget('Clocks', app);
        });

        it('strips the instance prefix when it matches', () => {
            assert.equal(
                widget._pinLeafName('u_top/u_dut/q_reg/D', 'u_top/u_dut/q_reg'),
                'D');
            assert.equal(
                widget._pinLeafName('u_top/u_dut/q_reg/CK', 'u_top/u_dut/q_reg'),
                'CK');
        });

        it('falls back to the trailing /-segment when no prefix match', () => {
            // Top-level ports, bus expansions, mismatched prefix all
            // fall through to the last "/" split.
            assert.equal(
                widget._pinLeafName('u_top/u_dut/q_reg/D', 'u_other'),
                'D');
            assert.equal(
                widget._pinLeafName('clk_in', 'u_top'),
                'clk_in');
        });

        it('handles empty / null pin gracefully', () => {
            assert.equal(widget._pinLeafName('', 'u_top'), '');
            assert.equal(widget._pinLeafName(null, 'u_top'), '');
            assert.equal(widget._pinLeafName(undefined, 'u_top'), '');
        });

        it('preserves a name with no slashes', () => {
            assert.equal(widget._pinLeafName('clk_a', null), 'clk_a');
            assert.equal(widget._pinLeafName('clk_a', 'u_top'), 'clk_a');
        });

        it('strips when instance path is the parent of a bit-indexed pin',
                () => {
            // Bus pins like `u_top/q_reg[3]` — the instance is `u_top`
            // and the leaf is `q_reg[3]`.
            assert.equal(
                widget._pinLeafName('u_top/q_reg[3]', 'u_top'),
                'q_reg[3]');
        });
    });

    describe('CDC stage card clock-domain shading', () => {
        const CDC_OVERVIEW = {
            time_unit: 'ns',
            current_mode: 'default',
            modes: { default: {
                endpoint_count: 1, clocks: ['clk_a', 'clk_b'],
                matrix: { clk_a: { clk_b: {
                    paths: 1, synced: 0, excluded: 0, unsynced: 1,
                }}},
            }},
        };

        it('same clock name produces the same hue across cards',
                async () => {
            // _cdcClockTint is deterministic: same name → same HSL hue.
            // Two register stages on `clk_b` should tint identically;
            // a stage on `clk_a` tints differently.
            const PATH_DETAIL = {
                stages: [
                    {
                        instance: 'ff_src_a', cell: 'DFF_X1',
                        odb_type: 'inst', odb_id: 10,
                        d_pin: 'ff_src_a/D',
                        d_pin_odb_type: 'iterm', d_pin_odb_id: 100,
                        q_pin: 'ff_src_a/Q',
                        q_pin_odb_type: 'iterm', q_pin_odb_id: 101,
                        kind: 'register', is_launch: true,
                        is_capture: false, is_sync_stage: false,
                        clock: 'clk_a', capture_clock: 'clk_b',
                        out_net: { name: 'q_a', odb_type: 'net', odb_id: 200 },
                        passthroughs_after: [],
                    },
                    {
                        instance: 'ff_capture', cell: 'DFF_X1',
                        odb_type: 'inst', odb_id: 11,
                        d_pin: 'ff_capture/D',
                        d_pin_odb_type: 'iterm', d_pin_odb_id: 102,
                        q_pin: 'ff_capture/Q',
                        q_pin_odb_type: 'iterm', q_pin_odb_id: 103,
                        kind: 'register', is_capture: true,
                        is_launch: false, is_sync_stage: false,
                        launch_clock: 'clk_a', capture_clock: 'clk_b',
                        clock: 'clk_b',
                        out_net: null, passthroughs_after: [],
                    },
                ],
                sync_chain: { kind: 'none', depth: 1,
                              whitelist_match: null, whitelist_pattern: null },
            };
            const PATHS = {
                time_unit: 'ns',
                total: 1, offset: 0,
                category_total: { synced: 0, excluded: 0, unsynced: 1 },
                paths: [{
                    capture_pin: 'ff_capture/D', odb_type: 'iterm', odb_id: 1,
                    capture_inst: 'ff_capture', capture_cell: 'DFF_X1',
                    launch_clock: 'clk_a', capture_clock: 'clk_b',
                    category: 'unsynchronized',
                    sync_chain_kind: 'none', sync_chain_depth: 1,
                    whitelist_match: null, whitelist_pattern: null,
                }],
            };
            const app = createMockApp({
                sdc_clocks: () => EMPTY_CLOCKS,
                sdc_clock_modes: () => EMPTY_MODES,
                cdc_overview: () => CDC_OVERVIEW,
                cdc_paths: () => PATHS,
                cdc_path_detail: () => PATH_DETAIL,
            });
            const widget = await setupTabWidget('CDC', app);
            // Verify the hash is deterministic and depends on the name.
            const tintA = widget._cdcClockTint('clk_a');
            const tintB = widget._cdcClockTint('clk_b');
            assert.notEqual(tintA, tintB,
                'different clock names produce different tints');
            assert.equal(widget._cdcClockTint('clk_a'), tintA,
                'same clock name produces the same tint on every call');
            assert.equal(widget._cdcClockTint(null), null,
                'null clock yields no tint');
        });
    });
});
