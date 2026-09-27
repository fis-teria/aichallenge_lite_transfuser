"use strict";

// A persistent desktop sidebar becomes a modal drawer on narrow screens.
const sidebar = document.querySelector("#project-menu");
const menuToggle = document.querySelector(".menu-toggle");
const menuClose = document.querySelector(".menu-close");
const backdrop = document.querySelector(".menu-backdrop");
const mainContent = document.querySelector("main");
const topbar = document.querySelector(".topbar");
const narrowScreen = window.matchMedia("(max-width: 900px)");

function setMenuOpen(open, restoreFocus = true) {
  const expanded = open && narrowScreen.matches;
  document.body.classList.toggle("menu-open", expanded);
  menuToggle.setAttribute("aria-expanded", String(expanded));
  backdrop.hidden = !expanded;
  sidebar.inert = narrowScreen.matches && !expanded;
  mainContent.inert = expanded;
  topbar.inert = expanded;
  if (expanded) {
    sidebar.setAttribute("role", "dialog");
    sidebar.setAttribute("aria-modal", "true");
    menuClose.focus();
  } else {
    sidebar.removeAttribute("role");
    sidebar.removeAttribute("aria-modal");
    if (restoreFocus && narrowScreen.matches) menuToggle.focus();
  }
}

document.documentElement.classList.add("navigation-ready");
setMenuOpen(false, false);
menuToggle.addEventListener("click", () => setMenuOpen(true));
menuClose.addEventListener("click", () => setMenuOpen(false));
backdrop.addEventListener("click", () => setMenuOpen(false));
sidebar.addEventListener("click", (event) => {
  const link = event.target.closest("a[href^='#']");
  if (!link || !narrowScreen.matches) return;
  setMenuOpen(false, false);
  const destination = document.querySelector(link.hash);
  if (destination) {
    destination.setAttribute("tabindex", "-1");
    destination.focus({ preventScroll: true });
  }
});
document.addEventListener("keydown", (event) => {
  if (!document.body.classList.contains("menu-open")) return;
  if (event.key === "Escape") {
    event.preventDefault();
    setMenuOpen(false);
  } else if (event.key === "Tab") {
    const focusable = [...sidebar.querySelectorAll("button, a[href]")];
    const first = focusable[0];
    const last = focusable[focusable.length - 1];
    if (event.shiftKey && document.activeElement === first) {
      event.preventDefault();
      last.focus();
    } else if (!event.shiftKey && document.activeElement === last) {
      event.preventDefault();
      first.focus();
    }
  }
});
narrowScreen.addEventListener("change", () => {
  const focusWasInSidebar = sidebar.contains(document.activeElement);
  setMenuOpen(false, false);
  if (focusWasInSidebar) {
    const nextFocus = narrowScreen.matches
      ? menuToggle
      : sidebar.querySelector("nav a[aria-current]");
    nextFocus.focus();
  }
});

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
  const marker = narrowScreen.matches ? 100 : 120;
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
