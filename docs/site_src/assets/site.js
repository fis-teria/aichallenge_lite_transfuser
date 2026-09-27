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
  const link = event.target.closest("a[href]");
  if (!link || !narrowScreen.matches) return;
  setMenuOpen(false, false);
  const destination =
    link.hash && link.pathname === location.pathname
      ? document.querySelector(link.hash)
      : null;
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

// Preserve bookmarks from the original single-page notebook.
if (document.body.dataset.page === "home") {
  const routes = {
    "#architecture": "articles/system-architecture.html",
    "#results": "results.html",
    "#stack": "articles/technology-stack.html",
    "#papers": "papers.html",
    "#next": "articles/roadmap.html",
  };
  if (routes[location.hash])
    location.replace(new URL(routes[location.hash], location.href));
}
// The full catalog remains readable without JavaScript.
const search = document.querySelector("#article-search");
if (search) {
  const buttons = [...document.querySelectorAll("[data-filter]")];
  const articles = [...document.querySelectorAll(".article-card")];
  const status = document.querySelector(".search-status");
  const normalize = (value) => value.normalize("NFKC").toLocaleLowerCase("ja");
  let category = "all";
  function filter() {
    const terms = normalize(search.value).trim().split(/\s+/u).filter(Boolean);
    let count = 0;
    for (const article of articles) {
      article.hidden =
        !(category === "all" || article.dataset.category === category) ||
        !terms.every((term) => normalize(article.textContent).includes(term));
      if (!article.hidden) count += 1;
    }
    status.textContent = `${articles.length} 件中 ${count} 件を表示`;
    document.querySelector("#article-empty").hidden = count !== 0;
  }
  document.querySelector(".paper-tools").hidden = false;
  status.hidden = false;
  search.addEventListener("input", filter);
  for (const button of buttons)
    button.addEventListener("click", () => {
      category = button.dataset.filter;
      for (const item of buttons)
        item.setAttribute("aria-pressed", String(item === button));
      filter();
    });
  filter();
}
