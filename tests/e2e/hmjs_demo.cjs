// The hmjs demo (https://github.com/tomquist/hmjs) in Chromium, connected to an
// emulated Venus E 3.0 over marsemu's BLE WebSocket. Prints one JSON line with
// what the page showed; tests/test_firmware.py checks it against the plant.
//
//   node hmjs_demo.cjs <demo URL> <ws URL>
const { chromium } = require("playwright");
const [demoUrl, wsUrl] = process.argv.slice(2);

(async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1100, height: 1400 } });
  const errors = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto(`${demoUrl}?emulator=${encodeURIComponent(wsUrl)}`);
  await page.locator(".disclaimer-checkbox input").check();
  await page.getByRole("button", { name: "Continue" }).click();
  await page.getByRole("button", { name: /Scan for Device/ }).click();
  await page.getByText(/Connected to MST_/).first().waitFor({ timeout: 60000 });
  const out = { device: await page.locator("header p").innerText() };
  await page.getByRole("button", { name: /^Status$/ }).click();
  const value = (label) =>
    page.locator(".info-row", { hasText: label }).first().locator(".info-value").innerText();
  await page.getByRole("button", { name: "Read status" }).click();
  await page.getByText("Depth of discharge:").first().waitFor();
  out.ac = await value("AC power");
  out.soc = await value("State of charge");
  await page.getByRole("button", { name: "Read battery and cells" }).click();
  await page.getByText("Cell voltages:").first().waitFor();
  out.cells = await value("Cell voltages");
  await page.locator("#family-dod").fill("80");
  await page
    .locator(".form-group", { has: page.locator("#family-dod") })
    .getByRole("button", { name: "Set" })
    .click();
  await page.waitForTimeout(2000);
  await page.getByRole("button", { name: "Read status" }).click();
  await page.waitForTimeout(1500);
  out.dod = await value("Depth of discharge");
  out.errors = errors;
  console.log(JSON.stringify(out));
  await browser.close();
})().catch((e) => {
  console.error(e.message);
  process.exit(1);
});
