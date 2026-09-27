"use strict";

// Progressive enhancement: the complete document is readable without JavaScript.
const paperSearch = document.querySelector("#paper-search");
const filterButtons = [...document.querySelectorAll("[data-filter]")];
const papers = [...document.querySelectorAll(".paper")];
const searchStatus = document.querySelector(".search-status");
let selectedCategory = "all";
const normalize = (value) => value.normalize("NFKC").toLocaleLowerCase("ja");

function filterPapers() {
  const terms = normalize(paperSearch.value)
    .trim()
    .split(/\s+/u)
    .filter(Boolean);
  let visible = 0;
  for (const paper of papers) {
    const categoryMatches =
      selectedCategory === "all" || paper.dataset.category === selectedCategory;
    const text = normalize(paper.textContent);
    paper.hidden =
      !categoryMatches || !terms.every((term) => text.includes(term));
    if (!paper.hidden) visible += 1;
  }
  searchStatus.textContent = `${papers.length} 件中 ${visible} 件を表示`;
  document.querySelector("#paper-empty").hidden = visible !== 0;
}

document.querySelector(".paper-tools").hidden = false;
searchStatus.hidden = false;
paperSearch.addEventListener("input", filterPapers);
for (const button of filterButtons) {
  button.addEventListener("click", () => {
    selectedCategory = button.dataset.filter;
    for (const item of filterButtons)
      item.setAttribute("aria-pressed", String(item === button));
    filterPapers();
  });
}
filterPapers();

// Keep the section marker consistent with the reader's scroll position.
const navigation = [...document.querySelectorAll("nav a[href^='#']")];
const sections = navigation.map((link) =>
  document.querySelector(link.getAttribute("href")),
);
let scheduled = false;
function updateNavigation() {
  const marker = window.matchMedia("(max-width: 900px)").matches ? 170 : 120;
  let active = sections[0];
  for (const section of sections) {
    if (section.getBoundingClientRect().top <= marker) active = section;
  }
  for (const link of navigation) {
    if (link.hash === `#${active.id}`)
      link.setAttribute("aria-current", "location");
    else link.removeAttribute("aria-current");
  }
  scheduled = false;
}
function scheduleNavigation() {
  if (!scheduled) {
    scheduled = true;
    window.requestAnimationFrame(updateNavigation);
  }
}
window.addEventListener("scroll", scheduleNavigation, { passive: true });
window.addEventListener("resize", scheduleNavigation);
window.addEventListener("hashchange", scheduleNavigation);
updateNavigation();
